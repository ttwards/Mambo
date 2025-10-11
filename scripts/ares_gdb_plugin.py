import gdb


class PrintExpressionCommand(gdb.Command):
    """
    typedef struct AresGdbPlotRequest {
      void *address;
      enum {
          TYPE_UNKNOWN,
          TYPE_U8,
          TYPE_U16,
          TYPE_U32,
          TYPE_I8,
          TYPE_I16,
          TYPE_I32,
          TYPE_F32,
          TYPE_F64
        } type;
      char name[32];
    } AresGdbPlotRequest;
    AresGdbPlotRequest aresGdbPlotReq[CONFIG_MAX_PLOTS];
    """

    def __init__(self):
        try:
            super(PrintExpressionCommand, self).__init__("plot", gdb.COMMAND_USER)
            self.gdbReqSymbol = "aresPlotData"
            reqStruct = gdb.parse_and_eval(self.gdbReqSymbol)
            fdata = reqStruct["fdata"]

            # 获取数组类型信息
            array_type = fdata.type
            element_size = array_type.target().sizeof  # float大小
            total_size = array_type.sizeof  # 总大小
            self.reqArrayLen = total_size // element_size  # 数组长度

            print("there are {} available plot lines".format(self.reqArrayLen))
            self.reqArray = reqStruct
        except gdb.error as e:
            print(f"Error: {e}")

    def parseType(self, type):
        if type.code == gdb.TYPE_CODE_INT and type.sizeof == 1 and not type.is_signed:
            return 5
        if type.code == gdb.TYPE_CODE_INT and type.sizeof == 2 and not type.is_signed:
            return 6
        if type.code == gdb.TYPE_CODE_INT and type.sizeof == 4 and not type.is_signed:
            return 7
        if type.code == gdb.TYPE_CODE_INT and type.sizeof == 1 and type.is_signed:
            return 3
        if type.code == gdb.TYPE_CODE_INT and type.sizeof == 2 and type.is_signed:
            return 4
        if type.code == gdb.TYPE_CODE_INT and type.sizeof == 4 and type.is_signed:
            return 0
        if type.code == gdb.TYPE_CODE_FLT and type.sizeof == 4:
            return 1
        if type.code == gdb.TYPE_CODE_FLT and type.sizeof == 8:
            return 2
        return 0

    def storeRequest(self, name, address, type):
        try:
            # 获取结构体实例
            plot_data = self.reqArray

            print("Struct get.")

            # 获取当前channel值
            current_channel = int(plot_data["channel"])
            data_ptr_array = plot_data["data_ptr"]
            types_array = plot_data["types"]

            print(f"current_channel: {current_channel}")

            # 边界检查
            if current_channel >= self.reqArrayLen:
                print("Error: Channel array is full")
                return False

            # 设置data_ptr[channel]
            p = data_ptr_array[current_channel].address
            command = f"set {{unsigned long}} {int(p)} = {address}"
            print(command)
            gdb.execute(command)

            # 设置types[channel]
            p = types_array[current_channel].address
            command = f"set {{unsigned char}} {int(p)} = {self.parseType(type)}"
            print(command)
            gdb.execute(command)

            # 增加channel计数
            p = plot_data["channel"].address
            command = f"set {{unsigned long}} {int(p)} = {current_channel + 1}"
            print(command)
            gdb.execute(command)

            return True

        except gdb.error as e:
            print(f"Error: {e}")
            return False

    def invoke(self, arg, from_tty):
        try:
            # 获取表达式的值
            value = gdb.parse_and_eval(arg)

            # 获取表达式的地址
            address = int(value.address)

            # 获取表达式的数据类型
            data_type = value.type.strip_typedefs()

            # 打印结果
            print("Expression: {}".format(arg))
            print("Address: {}".format(hex(address)))
            print("Value: {}".format(value))
            print(
                "Data Type: {} which is {}".format(data_type, self.parseType(data_type))
            )
            self.storeRequest(arg, address, data_type)
        except gdb.error as e:
            print("Error: {}".format(e))


PrintExpressionCommand()
