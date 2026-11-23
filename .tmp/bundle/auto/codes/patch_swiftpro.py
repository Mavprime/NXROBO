#!/usr/bin/env python3
import sys

def patch_file(filepath):
    with open(filepath, "r") as f:
        content = f.read()

    old_block = """\ttry
\t{
\t\t_serial.setPort("/dev/ttyACM0");
\t\t_serial.setBaudrate(115200);
\t\tserial::Timeout to = serial::Timeout::simpleTimeout(1000);
\t\t_serial.setTimeout(to);
\t\t_serial.open();
\t\tROS_INFO_STREAM("Port has been open successfully");
\t}
\tcatch (serial::IOException& e)
\t{
\t\tROS_ERROR_STREAM("Unable to open port");
\t\treturn -1;
\t}"""

    new_block = """\tstd::string candidate_ports[] = {"/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2"};
\tbool port_opened = false;
\tfor (size_t i = 0; i < 3; i++)
\t{
\t\ttry
\t\t{
\t\t\t_serial.setPort(candidate_ports[i]);
\t\t\t_serial.setBaudrate(115200);
\t\t\tserial::Timeout to = serial::Timeout::simpleTimeout(1000);
\t\t\t_serial.setTimeout(to);
\t\t\t_serial.open();
\t\t\tif (_serial.isOpen())
\t\t\t{
\t\t\t\tROS_INFO_STREAM("Port " << candidate_ports[i] << " has been open successfully");
\t\t\t\tport_opened = true;
\t\t\t\tbreak;
\t\t\t}
\t\t}
\t\tcatch (serial::IOException& e)
\t\t{
\t\t}
\t}
\tif (!port_opened)
\t{
\t\tROS_ERROR_STREAM("Unable to open any port (/dev/ttyACM0, /dev/ttyACM1, /dev/ttyACM2)");
\t\treturn -1;
\t}"""

    if old_block in content:
        content = content.replace(old_block, new_block, 1)
        with open(filepath, "w") as f:
            f.write(content)
        print("Patched:", filepath)
        return True
    else:
        print("Already patched or pattern not found in:", filepath)
        return False

if __name__ == "__main__":
    p1 = patch_file("/home/spark/spark_noetic/src/spark_driver/arm/uArm/swiftpro/src/swiftpro_write_node.cpp")
    p2 = patch_file("/home/spark/spark_noetic/src/spark_driver/arm/uArm/swiftpro/src/swiftpro_read_node.cpp")
    if p1 or p2:
        sys.exit(10) # Signal that recompile is needed
    sys.exit(0)
