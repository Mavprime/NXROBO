"""``librknnrt.so`` 的 ctypes 绑定 —— rknn-toolkit-lite2 推理子集的兼容层。

为什么需要
----------
本板是 OpenHarmony（musl / OHOS ABI）：PyPI 上的 ``rknn-toolkit-lite2`` 只有
manylinux(glibc) 轮子，pip 一个标签都不认，装不上；厂商只提供了 C 运行时
``librknnrt.so``。C ABI 是稳定的，用 ctypes 直接绑即可 —— 不需要编译器、
不需要 glibc，这是 musl 板上唯一可行的路径。

暴露两个门面
------------
* :class:`RKNNLite` —— 对齐官方 ``rknn-toolkit-lite2`` 的 ``RKNNLite`` 接口
* :class:`RKNN`     —— 对齐 ``rknn_model_zoo`` 里 ``rknn.api.RKNN`` 的写法

分层约定
--------
本层只做「模型文件 → float32 输出数组」的搬运，**不含任何后处理假设**：
不猜 sigmoid、不猜 YOLO head 布局、不做解码。那些是调用方的职责
（``nodes/rknn_yolo.py``）—— 在这一层猜错会静默给出错误的框。

结构体字段顺序照抄 ``rknn_api.h``（v1.5/1.6 ABI）；与板上 ``librknnrt.so``
用真实推理验证过，不要"整理"字段顺序或省略字段（对齐坑见
``references/ctypes-api.md``）。
"""

import ctypes
import os
import warnings

import numpy as np

__all__ = [
    "RKNNLite",
    "RKNN",
    "TensorInfo",
    "DEFAULT_LIB",
    "NPU_CORE_AUTO",
    "NPU_CORE_0",
    "NPU_CORE_1",
    "NPU_CORE_2",
    "NPU_CORE_0_1",
    "NPU_CORE_0_1_2",
    "NPU_CORE_ALL",
]

# --------------------------------------------------------------------- 常量 ----

RKNN_MAX_DIMS = 16
RKNN_MAX_NAME_LEN = 256

# rknn_query_cmd
RKNN_QUERY_IN_OUT_NUM = 0
RKNN_QUERY_INPUT_ATTR = 1
RKNN_QUERY_OUTPUT_ATTR = 2
RKNN_QUERY_SDK_VERSION = 5

# rknn_tensor_type
RKNN_TENSOR_FLOAT32 = 0
RKNN_TENSOR_FLOAT16 = 1
RKNN_TENSOR_INT8 = 2
RKNN_TENSOR_UINT8 = 3
RKNN_TENSOR_INT16 = 4
RKNN_TENSOR_UINT16 = 5
RKNN_TENSOR_INT32 = 6
RKNN_TENSOR_UINT32 = 7
RKNN_TENSOR_INT64 = 8
RKNN_TENSOR_BOOL = 9
RKNN_TENSOR_INT4 = 10

# rknn_tensor_format
RKNN_TENSOR_NCHW = 0
RKNN_TENSOR_NHWC = 1

# rknn_core_mask —— RK3588 有三个 NPU 核。不给 mask 时运行时只用 core0，
# 这就是"Core1/Core2 占用率恒为 0"的原因（见 references/performance.md）。
NPU_CORE_AUTO = 0
NPU_CORE_0 = 1
NPU_CORE_1 = 2
NPU_CORE_2 = 4
NPU_CORE_0_1 = 3
NPU_CORE_0_1_2 = 7
NPU_CORE_ALL = 0xFFFF

# rknn_api.h 里的错误码
RKNN_ERR_NAMES = {
    0: "RKNN_SUCC",
    -1: "RKNN_ERR_FAIL",
    -2: "RKNN_ERR_TIMEOUT",
    -3: "RKNN_ERR_DEVICE_UNAVAILABLE",
    -4: "RKNN_ERR_MALLOC_FAIL",
    -5: "RKNN_ERR_PARAM_INVALID",
    -6: "RKNN_ERR_MODEL_INVALID",
    -7: "RKNN_ERR_CTX_INVALID",
    -8: "RKNN_ERR_INPUT_INVALID",
    -9: "RKNN_ERR_OUTPUT_INVALID",
    -10: "RKNN_ERR_DEVICE_UNMATCH",
    -11: "RKNN_ERR_INCOMPATILE_PRE_COMPILE_MODEL",
    -12: "RKNN_ERR_INCOMPATILE_OPTIMIZATION_LEVEL_VERSION",
    -13: "RKNN_ERR_TARGET_PLATFORM_UNMATCH",
}

# 与本板（KaiHong RK3588 / OpenHarmony）厂商运行时一致的默认路径。
DEFAULT_LIB = "/data/local/release/rknn-demo/lib/librknnrt.so"

_DATA_TYPE_TO_TENSOR = {
    "float32": RKNN_TENSOR_FLOAT32,
    "float16": RKNN_TENSOR_FLOAT16,
    "int8": RKNN_TENSOR_INT8,
    "uint8": RKNN_TENSOR_UINT8,
    "int16": RKNN_TENSOR_INT16,
    "uint16": RKNN_TENSOR_UINT16,
    "int32": RKNN_TENSOR_INT32,
    "uint32": RKNN_TENSOR_UINT32,
    "int64": RKNN_TENSOR_INT64,
    "bool": RKNN_TENSOR_BOOL,
}

_NUMPY_TO_DATA_TYPE = {
    "uint8": "uint8",
    "int8": "int8",
    "float32": "float32",
    "float16": "float16",
    "int16": "int16",
    "uint16": "uint16",
    "int32": "int32",
    "uint32": "uint32",
    "int64": "int64",
}

_DATA_FORMAT_TO_TENSOR = {"nchw": RKNN_TENSOR_NCHW, "nhwc": RKNN_TENSOR_NHWC}


def error_name(ret):
    """把 rknn 返回码翻成人能读的名字（未收录的返回码原样回显）。"""
    return RKNN_ERR_NAMES.get(ret, "RKNN_ERR_UNKNOWN(%d)" % ret)


# ------------------------------------------------------------- C 结构体镜像 ----


class RknnTensorAttr(ctypes.Structure):
    _fields_ = [
        ("index", ctypes.c_uint32),
        ("n_dims", ctypes.c_uint32),
        ("dims", ctypes.c_uint32 * RKNN_MAX_DIMS),
        ("name", ctypes.c_char * RKNN_MAX_NAME_LEN),
        ("n_elems", ctypes.c_uint32),
        ("size", ctypes.c_uint32),
        ("fmt", ctypes.c_int),
        ("type", ctypes.c_int),
        ("qnt_type", ctypes.c_int),
        ("fl", ctypes.c_int8),
        ("zp", ctypes.c_int32),
        ("scale", ctypes.c_float),
        ("w_stride", ctypes.c_uint32),
        ("size_with_stride", ctypes.c_uint32),
        ("pass_through", ctypes.c_uint8),
        ("h_stride", ctypes.c_uint32),
    ]


class RknnInput(ctypes.Structure):
    _fields_ = [
        ("index", ctypes.c_uint32),
        ("buf", ctypes.c_void_p),
        ("size", ctypes.c_uint32),
        ("pass_through", ctypes.c_uint8),
        ("type", ctypes.c_int),
        ("fmt", ctypes.c_int),
    ]


class RknnOutput(ctypes.Structure):
    _fields_ = [
        ("want_float", ctypes.c_uint8),
        ("is_prealloc", ctypes.c_uint8),
        ("index", ctypes.c_uint32),
        ("buf", ctypes.c_void_p),
        ("size", ctypes.c_uint32),
    ]


class RknnInputOutputNum(ctypes.Structure):
    _fields_ = [("n_input", ctypes.c_uint32), ("n_output", ctypes.c_uint32)]


class RknnSdkVersion(ctypes.Structure):
    _fields_ = [
        ("api_version", ctypes.c_char * 256),
        ("drv_version", ctypes.c_char * 256),
    ]


class TensorInfo(object):
    """rknn_query 回来的张量描述，只留调用方用得上的字段。"""

    __slots__ = ("index", "name", "shape", "fmt", "type", "n_elems", "size", "zp", "scale")

    def __init__(self, attr):
        self.index = int(attr.index)
        self.name = attr.name.decode("utf-8", "replace")
        self.shape = [int(attr.dims[d]) for d in range(attr.n_dims)]
        self.fmt = int(attr.fmt)
        self.type = int(attr.type)
        self.n_elems = int(attr.n_elems)
        self.size = int(attr.size)
        self.zp = int(attr.zp)
        self.scale = float(attr.scale)

    @property
    def is_nhwc(self):
        return self.fmt == RKNN_TENSOR_NHWC

    def __repr__(self):
        return "TensorInfo(index=%d, name=%r, shape=%s, fmt=%d, type=%d)" % (
            self.index, self.name, self.shape, self.fmt, self.type,
        )


# ----------------------------------------------------------------- 动态库加载 ----

_lib_cache = {}


def _preload_cxx_runtime(lib_path):
    """先 dlopen ``libc++_shared.so`` 再 dlopen ``librknnrt.so``。

    ``librknnrt.so`` 依赖 ``libc++_shared.so``，而 OpenHarmony release 里
    没有把它装到系统库路径；板上没有 ``ldd``，与其猜依赖不如把厂商放在
    运行时旁边的那个副本直接预加载（RTLD_GLOBAL 让后续 dlopen 能解析到）。
    """
    candidates = []
    env = os.environ.get("RKNN_CXX_LIB")
    if env:
        candidates.append(env)
    lib_dir = os.path.dirname(os.path.abspath(lib_path))
    candidates.append(os.path.join(lib_dir, "libc++_shared.so"))
    candidates.append("/data/local/release/rknn-demo/lib/libc++_shared.so")
    candidates.append("/usr/lib/libc++_shared.so")
    for path in candidates:
        if path and os.path.exists(path):
            try:
                ctypes.CDLL(path, mode=ctypes.RTLD_GLOBAL)
            except OSError:
                # 已经加载过、或与本次运行时不同代：交给下一步 dlopen 报错，
                # 不要在这里吞掉真正的原因。
                continue
            return path
    return None


def _load_library(lib_path):
    """按路径缓存地 dlopen librknnrt.so 并声明函数签名。"""
    if lib_path in _lib_cache:
        return _lib_cache[lib_path]

    if not os.path.exists(lib_path):
        raise OSError(
            "找不到 RKNN 运行时 %r；用 lib_path= 或环境变量 RKNN_RT_LIB 指定 "
            "librknnrt.so 的位置" % lib_path
        )

    _preload_cxx_runtime(lib_path)
    try:
        lib = ctypes.CDLL(lib_path, mode=ctypes.RTLD_GLOBAL)
    except OSError as exc:
        raise OSError(
            "dlopen(%s) 失败: %s —— 通常是缺 libc++_shared.so（设 RKNN_CXX_LIB 指向它）"
            % (lib_path, exc)
        )

    # argtypes 必须显式声明：rknn_context 是 uint64，靠默认的 int 会被截断。
    lib.rknn_init.argtypes = [
        ctypes.POINTER(ctypes.c_uint64), ctypes.c_void_p,
        ctypes.c_uint32, ctypes.c_uint32, ctypes.c_void_p,
    ]
    lib.rknn_init.restype = ctypes.c_int
    lib.rknn_query.argtypes = [ctypes.c_uint64, ctypes.c_int, ctypes.c_void_p, ctypes.c_uint32]
    lib.rknn_query.restype = ctypes.c_int
    lib.rknn_inputs_set.argtypes = [ctypes.c_uint64, ctypes.c_uint32, ctypes.POINTER(RknnInput)]
    lib.rknn_inputs_set.restype = ctypes.c_int
    lib.rknn_run.argtypes = [ctypes.c_uint64, ctypes.c_void_p]
    lib.rknn_run.restype = ctypes.c_int
    lib.rknn_outputs_get.argtypes = [
        ctypes.c_uint64, ctypes.c_uint32, ctypes.POINTER(RknnOutput), ctypes.c_void_p,
    ]
    lib.rknn_outputs_get.restype = ctypes.c_int
    lib.rknn_outputs_release.argtypes = [
        ctypes.c_uint64, ctypes.c_uint32, ctypes.POINTER(RknnOutput),
    ]
    lib.rknn_outputs_release.restype = ctypes.c_int
    lib.rknn_destroy.argtypes = [ctypes.c_uint64]
    lib.rknn_destroy.restype = ctypes.c_int

    # 1.4 之前的运行时没有 set_core_mask；有则声明，没有就跳过。
    if hasattr(lib, "rknn_set_core_mask"):
        lib.rknn_set_core_mask.argtypes = [ctypes.c_uint64, ctypes.c_int]
        lib.rknn_set_core_mask.restype = ctypes.c_int

    _lib_cache[lib_path] = lib
    return lib


# ------------------------------------------------------------------- 运行时 ----

_CORE_MASK_NAMES = {
    NPU_CORE_AUTO: "auto",
    NPU_CORE_0: "0",
    NPU_CORE_1: "1",
    NPU_CORE_2: "2",
    NPU_CORE_0_1: "0_1",
    NPU_CORE_0_1_2: "0_1_2",
    NPU_CORE_ALL: "all",
}


def core_mask_from_string(value):
    """把 ``"0_1_2"`` / ``"0,1,2"`` / int 统一成 rknn_core_mask 位掩码。

    ROS 参数只能是字符串或整数，这里同时容忍 ``"NPU_CORE_0_1_2"`` 这种
    从官方示例里抄来的写法。
    """
    if isinstance(value, int):
        return value
    text = str(value).strip().upper()
    if text.startswith("NPU_CORE_"):
        text = text[len("NPU_CORE_"):]
    text = text.replace(",", "_").replace("|", "_").replace("+", "_")
    if text in ("AUTO", ""):
        return NPU_CORE_AUTO
    if text == "ALL":
        return NPU_CORE_ALL
    mask = 0
    for part in text.split("_"):
        if part == "":
            continue
        if part not in ("0", "1", "2"):
            raise ValueError(
                "无法识别的 core_mask %r；可用: auto / 0 / 1 / 2 / 0_1 / 0_1_2 / all" % value
            )
        mask |= {0: NPU_CORE_0, 1: NPU_CORE_1, 2: NPU_CORE_2}[int(part)]
    return mask


class _Runtime(object):
    """两个门面共用的引擎：模型 → float32 输出。"""

    def __init__(self, lib_path=None, verbose=False):
        self.verbose = verbose
        self.lib_path = lib_path or os.environ.get("RKNN_RT_LIB") or DEFAULT_LIB
        self._lib = _load_library(self.lib_path)
        self._ctx = None
        self._model_path = None
        self.n_input = 0
        self.n_output = 0
        self.input_attrs = []
        self.output_attrs = []
        self.sdk_version = None
        self.driver_version = None
        self.core_mask = NPU_CORE_AUTO

    # -- 生命周期 ------------------------------------------------------
    def load_rknn(self, path, file_size=0):
        if not os.path.exists(path):
            raise FileNotFoundError("模型文件不存在: %s" % path)
        self._model_path = path
        return 0

    def init_runtime(self, target=None, device_id=None, perf_debug=False,
                     eval_mem=False, async_mode=False, core_mask=NPU_CORE_AUTO,
                     **kwargs):
        if self._model_path is None:
            raise RuntimeError("必须先 load_rknn(path) 再 init_runtime()")
        # target/device_id/async_mode 在单板本地推理里没有对应参数：留着是为了
        # 让从官方示例抄来的调用不用改。
        if async_mode:
            warnings.warn("本兼容层不支持 async_mode，按同步推理执行")
        del target, device_id, perf_debug, eval_mem, kwargs

        ctx = ctypes.c_uint64(0)
        # size=0 表示 model 传的是"模型文件路径"，不是内存里的模型数据。
        ret = self._lib.rknn_init(
            ctypes.byref(ctx), ctypes.c_char_p(self._model_path.encode()), 0, 0, None
        )
        if ret != 0:
            raise RuntimeError(
                "rknn_init(%s) 失败: %s —— 最常见的原因是模型与运行时不同代"
                % (self._model_path, error_name(ret))
            )
        self._ctx = ctx

        ver = RknnSdkVersion()
        if self._lib.rknn_query(ctx, RKNN_QUERY_SDK_VERSION,
                                ctypes.byref(ver), ctypes.sizeof(ver)) == 0:
            self.sdk_version = ver.api_version.decode("utf-8", "replace")
            self.driver_version = ver.drv_version.decode("utf-8", "replace")

        self.core_mask = core_mask_from_string(core_mask)
        if self.core_mask != NPU_CORE_AUTO:
            if not hasattr(self._lib, "rknn_set_core_mask"):
                warnings.warn("当前 librknnrt.so 没有 rknn_set_core_mask，core_mask 被忽略")
            else:
                ret = self._lib.rknn_set_core_mask(self._ctx, self.core_mask)
                if ret != 0:
                    warnings.warn("rknn_set_core_mask(%d) 失败: %s"
                                  % (self.core_mask, error_name(ret)))

        io_num = RknnInputOutputNum()
        ret = self._lib.rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM,
                                   ctypes.byref(io_num), ctypes.sizeof(io_num))
        if ret != 0:
            raise RuntimeError("rknn_query(IN_OUT_NUM) 失败: %s" % error_name(ret))
        self.n_input, self.n_output = int(io_num.n_input), int(io_num.n_output)

        self.input_attrs = self._query_attrs(RKNN_QUERY_INPUT_ATTR, self.n_input)
        self.output_attrs = self._query_attrs(RKNN_QUERY_OUTPUT_ATTR, self.n_output)

        if self.verbose:
            print("rknn: api %s / driver %s / core_mask %s"
                  % (self.sdk_version, self.driver_version,
                     _CORE_MASK_NAMES.get(self.core_mask, self.core_mask)))
            for info in self.input_attrs:
                print("  in  %s" % info)
            for info in self.output_attrs:
                print("  out %s" % info)
        return 0

    def _query_attrs(self, cmd, count):
        attrs = []
        for i in range(count):
            attr = RknnTensorAttr()
            attr.index = i
            ret = self._lib.rknn_query(self._ctx, cmd, ctypes.byref(attr), ctypes.sizeof(attr))
            if ret != 0:
                raise RuntimeError("rknn_query(%d) 失败: %s" % (cmd, error_name(ret)))
            attrs.append(TensorInfo(attr))
        return attrs

    # -- 推理 ----------------------------------------------------------
    def inference(self, inputs, data_type=None, data_format="nhwc", inputs_pass_through=None):
        if self._ctx is None:
            raise RuntimeError("必须先 init_runtime()")
        if not isinstance(inputs, (list, tuple)):
            inputs = [inputs]
        if len(inputs) != max(self.n_input, 1):
            raise ValueError("模型需要 %d 个输入，收到 %d 个" % (self.n_input, len(inputs)))
        if data_format not in _DATA_FORMAT_TO_TENSOR:
            raise ValueError("data_format 只支持 nchw / nhwc，收到 %r" % (data_format,))
        fmt = _DATA_FORMAT_TO_TENSOR[data_format]

        if inputs_pass_through is None:
            pass_through = [0] * len(inputs)
        elif isinstance(inputs_pass_through, (list, tuple)):
            pass_through = [int(v) for v in inputs_pass_through]
        else:
            pass_through = [int(inputs_pass_through)] * len(inputs)

        rknn_inputs = (RknnInput * len(inputs))()
        # 这些数组必须活到 rknn_inputs_set 返回为止，否则 buf 指向已释放内存。
        held = []
        for i, arr in enumerate(inputs):
            buf = np.ascontiguousarray(arr)
            held.append(buf)
            if data_type is None:
                dtype_name = _NUMPY_TO_DATA_TYPE.get(buf.dtype.name)
                if dtype_name is None:
                    raise ValueError("不支持的输入 dtype %r；请显式给 data_type=" % (buf.dtype,))
            else:
                dtype_name = str(data_type).lower()
            if dtype_name not in _DATA_TYPE_TO_TENSOR:
                raise ValueError("不支持的 data_type %r" % (data_type,))
            rknn_inputs[i].index = i
            rknn_inputs[i].buf = buf.ctypes.data_as(ctypes.c_void_p)
            rknn_inputs[i].size = buf.nbytes
            # pass_through=0：前处理（mean/std、rgb2bgr）交给驱动按模型元数据做。
            rknn_inputs[i].pass_through = pass_through[i]
            rknn_inputs[i].type = _DATA_TYPE_TO_TENSOR[dtype_name]
            rknn_inputs[i].fmt = fmt

        ret = self._lib.rknn_inputs_set(self._ctx, len(inputs), rknn_inputs)
        if ret != 0:
            raise RuntimeError("rknn_inputs_set 失败: %s" % error_name(ret))

        ret = self._lib.rknn_run(self._ctx, None)
        if ret != 0:
            raise RuntimeError("rknn_run 失败: %s" % error_name(ret))

        outputs = (RknnOutput * self.n_output)()
        for i in range(self.n_output):
            outputs[i].want_float = 1     # 驱动负责反量化成 float32
            outputs[i].is_prealloc = 0    # 由驱动分配，用完必须 release
            outputs[i].index = i

        ret = self._lib.rknn_outputs_get(self._ctx, self.n_output, outputs, None)
        if ret != 0:
            raise RuntimeError("rknn_outputs_get 失败: %s" % error_name(ret))

        result = []
        try:
            for i in range(self.n_output):
                info = self.output_attrs[i]
                if not outputs[i].buf:
                    raise RuntimeError("输出 %d 的缓冲区为空" % i)
                flat = np.ctypeslib.as_array(
                    ctypes.cast(outputs[i].buf, ctypes.POINTER(ctypes.c_float)),
                    shape=(info.n_elems,),
                ).copy()    # 必须 copy：下面 release 之后这块内存就无效了
                arr = flat.reshape(info.shape)
                if info.is_nhwc:
                    # 统一成 NCHW：YOLO 系列的后处理都按 channel-first 写。
                    arr = np.transpose(arr, (0, 3, 1, 2))
                result.append(arr)
        finally:
            self._lib.rknn_outputs_release(self._ctx, self.n_output, outputs)
        return result

    def release(self):
        if self._ctx is not None:
            self._lib.rknn_destroy(self._ctx)
            self._ctx = None
        return 0


# --------------------------------------------------------------------- 门面 ----


class RKNNLite(_Runtime):
    """官方 ``rknn-toolkit-lite2.RKNNLite`` 的兼容实现（只含推理）。"""

    NPU_CORE_AUTO = NPU_CORE_AUTO
    NPU_CORE_0 = NPU_CORE_0
    NPU_CORE_1 = NPU_CORE_1
    NPU_CORE_2 = NPU_CORE_2
    NPU_CORE_0_1 = NPU_CORE_0_1
    NPU_CORE_0_1_2 = NPU_CORE_0_1_2
    NPU_CORE_ALL = NPU_CORE_ALL

    def __init__(self, verbose=False, lib_path=None):
        super(RKNNLite, self).__init__(lib_path=lib_path, verbose=verbose)

    def inference(self, inputs, data_type="float32", data_format="nhwc",
                  inputs_pass_through=None):
        # 官方 lite2 的默认 data_type 是 float32；uint8 输入要显式给。
        return super(RKNNLite, self).inference(
            inputs, data_type=data_type, data_format=data_format,
            inputs_pass_through=inputs_pass_through,
        )


class RKNN(_Runtime):
    """``rknn_model_zoo`` 里 ``rknn.api.RKNN`` 用法的兼容实现。

    与 :class:`RKNNLite` 的差别只在默认参数：zoo 的脚本习惯
    ``init_runtime(target="rk3588")`` 与 ``inference(inputs=[img])``。
    模型转换（load_onnx/build/export_rknn）是开发机上的操作，这里明确不支持。
    """

    NPU_CORE_AUTO = NPU_CORE_AUTO
    NPU_CORE_0 = NPU_CORE_0
    NPU_CORE_1 = NPU_CORE_1
    NPU_CORE_2 = NPU_CORE_2
    NPU_CORE_0_1 = NPU_CORE_0_1
    NPU_CORE_0_1_2 = NPU_CORE_0_1_2
    NPU_CORE_ALL = NPU_CORE_ALL

    def __init__(self, verbose=False, lib_path=None):
        super(RKNN, self).__init__(lib_path=lib_path, verbose=verbose)

    def inference(self, inputs, data_type="uint8", data_format="nhwc",
                  inputs_pass_through=None):
        # zoo 的 YOLO demo 一律喂 uint8 NHWC（前处理由驱动做）。
        return super(RKNN, self).inference(
            inputs, data_type=data_type, data_format=data_format,
            inputs_pass_through=inputs_pass_through,
        )

    # -- 只存在于开发机的模型转换接口：显式拒绝，别让人以为"跑通了" ----
    def _host_only(self, *args, **kwargs):
        raise NotImplementedError(
            "模型转换（load_onnx/load_tflite/load_pytorch/build/export_rknn）"
            "是开发机上 rknn-toolkit2 的职责，板上只有运行时"
        )

    load_onnx = _host_only
    load_tflite = _host_only
    load_pytorch = _host_only
    load_tensorflow = _host_only
    load_caffe = _host_only
    load_darknet = _host_only
    build = _host_only
    export_rknn = _host_only
    accuracy_analysis = _host_only
