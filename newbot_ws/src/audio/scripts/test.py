import pulsectl

def increase_volume():
    # 创建 PulseAudio 的客户端对象
    pulse = pulsectl.Pulse('volume-control')

    # 获取默认输出设备的音量信息
    sink_info = pulse.get_sink_by_name('@DEFAULT_SINK@')

    # 增加音量并设置到输出设备
    new_volume = min(sink_info.volume.value + (0x10000 * 0.2), 0x10000)
    pulse.volume_set_all_chans(sink_info, new_volume)

    # 关闭 PulseAudio 客户端连接
    pulse.close()

# 调用函数将音量增加20%
increase_volume()