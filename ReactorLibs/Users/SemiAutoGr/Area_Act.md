# MeiLinArray 类 使用说明

## 快速开始

### 工控机控制模式

功能码：`func_code = 0x91`，动作功能码组成`payload`发送

### 电控调试模式

最小调用模板如下。

```
uint8_t my_seq[] = 
{
    Area2Code::FETCH_UP200,   // 取上200的块
    Area2Code::UP_200,        // 上200
    Area2Code::UP_200,        // 上200
    Area2Code::UP_200,        // 上200
    Area2Code::DOWN_200,      // 下200
    Area2Code::DOWN_400,      // 下400
    Area2Code::AREA_DONE,     // 结束
};

Area2::mei_lin_array.SetAction(my_seq, sizeof(my_seq));
Area2::mei_lin_array.ExecuteSequence();
```

注意序列末尾要加上`Area2Code::AREA_DONE`