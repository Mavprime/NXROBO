"""``rknn`` 兼容层：在没有 rknn-toolkit-lite2 的 OpenHarmony 板子上提供推理接口。

本板是 musl / OHOS ABI，官方 ``rknn-toolkit-lite2`` 只有 glibc 轮子，装不上。
本包用 ctypes 绑定厂商的 ``librknnrt.so``，暴露官方 API 的推理子集：

    from rknn.api import RKNNLite

    rknn = RKNNLite()
    rknn.load_rknn("/path/model.rknn")
    rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2)
    outputs = rknn.inference(inputs=[bgr_or_rgb_uint8], data_type="uint8", data_format="nhwc")
    rknn.release()

安装位置与可见性
----------------
本包随 semi_auto_match_24 装在
``<prefix>/share/semi_auto_match_24/python/rknn/``，
由 ``nodes/rknn_yolo.py`` 自己把该目录塞进 ``sys.path``（见该模块的
``_ensure_rknn_importable``）。这样做而不是装进 site-packages，是为了：

* 不遮蔽将来真正安装的官方 ``rknn`` 包 —— 同名顶层模块一定会遮蔽，出问题很难查；
* 交叉编译期不需要 ``catkin_python_setup()``（那会在构建机上跑 setup.py）。

如果确实需要全局可见（例如要跑 rknn_model_zoo 的 demo），
参考 auto_match 包里的 ``scripts/install_rknn_shim.sh`` 显式安装，
并注意上述遮蔽风险。
"""

from .api import (  # noqa: F401
    RKNN,
    RKNNLite,
    TensorInfo,
    DEFAULT_LIB,
    NPU_CORE_0,
    NPU_CORE_0_1,
    NPU_CORE_0_1_2,
    NPU_CORE_1,
    NPU_CORE_2,
    NPU_CORE_ALL,
    NPU_CORE_AUTO,
    core_mask_from_string,
    error_name,
)

__version__ = "0.1.0"

# 官方包一旦装上，同名遮蔽就会发生；这里主动提示，而不是让它静默生效。
__shim__ = True

__all__ = [
    "RKNN",
    "RKNNLite",
    "TensorInfo",
    "DEFAULT_LIB",
    "NPU_CORE_AUTO",
    "NPU_CORE_0",
    "NPU_CORE_1",
    "NPU_CORE_2",
    "NPU_CORE_0_1",
    "NPU_CORE_0_1_2",
    "NPU_CORE_ALL",
    "core_mask_from_string",
    "error_name",
    "__version__",
]
