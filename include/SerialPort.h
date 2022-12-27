#ifndef __SERIALPORT_
#define __SERIALPORT_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>   //文件控制定义
#include <termios.h> //终端控制定义
#include <memory>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <vector>

#define S_TIMEOUT 1

class SerialPort
{
public:
  SerialPort(int baud_rate = B115200);

  ~SerialPort();

  void open();  // 打开串口
  void close(); // 关闭串口
  inline bool isOpen() { return is_open; };

  // TODO 使用头尾校验无法实现透明传输
  // 需要改进串口协议

  /**
   * @brief 从串口读取结构体
   * 
   * @tparam T 读取到结构体的类型
   * @param head 头帧
   * @param tail 尾帧
   * @return std::vector<T> 读到的所有结构体
   */
  template <typename T>
  std::vector<T> readStruct(unsigned char head, unsigned char tail)
  {
    std::vector<T> vec_t;
    const int LENGTH = 512;
    const int SIZE = sizeof(T);
    unsigned char read_buffer[LENGTH] = {0};
    ssize_t len_result = this->read(read_buffer, LENGTH);
    for (ssize_t i = 0; (i + SIZE + 1) < len_result; i++)
    {
      if (read_buffer[i] == head && read_buffer[i + SIZE + 1] == tail)
      {
        //TODO 引发 GCC 警告: -Wstrict-aliasing
        vec_t.push_back(*(reinterpret_cast<T *>(&read_buffer[i + 1])));
      }
    }
    return vec_t;
  }

  /**
   * @brief 结构体写入串口
   * 
   * @tparam T 写入结构体的类型
   * @param data_struct 要写入的结构体
   * @return 是否写入成功
   */
  template <typename T>
  bool writeStruct(T data_struct)
  {
    ssize_t len_result = this->write(&data_struct, sizeof(data_struct));
    return (sizeof(data_struct) == len_result);
  }

private:
  int fd;
  struct termios option;
  bool is_open = false;
  int baud_rate;

  ssize_t write(void *data, size_t len);
  ssize_t read(void *data, size_t len);
};

using serial_ptr = std::shared_ptr<SerialPort>;

#endif // !__SERIALPORT_
