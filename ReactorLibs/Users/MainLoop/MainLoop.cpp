#include "MainLoop.hpp"
#include "HostPC.hpp"
#include "tim.h"

MainLoop& APP::main_loop = MainLoop::GetInstance();

float debug_cnt = 0;

void MainLoop::Start()
{
    test_led.Init(Hardware::tim_led, BSP::TIM::CH4);
    test_led.BiasFactor = Vec3(0.843f, 1.0f, 0.843f);

    // IR 红外接收 (PA0, EXTI)
    ir_rx.Init(Pin{'A', 0});
    ir_rx.local_addr = 0xFF;  // 接收所有地址的帧
    ir_rx.RegisterRxHandler([](const IR_RX *rx,
                                uint8_t address, uint8_t command,
                                bool is_repeat, void *user_ctx) {
        MainLoop *self = (MainLoop *)user_ctx;
        self->_ir_rx_seq++;
        APP::monit.LogInfo("[IR_RX] #%d recv addr=0x%02X cmd=0x%02X repeat=%d",
                           self->_ir_rx_seq, address, command, is_repeat);
    }, this);

    // IR 红外发射 (TIM1_CH3 + PE12 CH3N, 38kHz)
    ir_tx.Init(ToID(&htim1), BSP::TIM::CH3);
    ir_tx.RegisterDoneCallback(_IrTxDoneCb, this);
}

void MainLoop::SendIR(uint8_t address, uint8_t command)
{
    _ir_tx_last_addr = address;
    _ir_tx_last_cmd  = command;

    if (!ir_tx.Send(address, command))
    {
        APP::monit.LogWarning("[IR_TX] #%d send FAILED (busy) addr=0x%02X cmd=0x%02X",
                              _ir_tx_seq + 1, address, command);
    }
    // DMA 完成后由 _IrTxDoneCb 打印成功日志
}

void MainLoop::_IrTxDoneCb(BSP::TIM::TimID tim, BSP::TIM::Channel ch, void *ctx)
{
    MainLoop *self = (MainLoop *)ctx;
    self->_ir_tx_seq++;
    // APP::monit.LogInfo("[IR_TX] #%d sent OK addr=0x%02X cmd=0x%02X",
    //                    self->_ir_tx_seq,
    //                    self->_ir_tx_last_addr,
    //                    self->_ir_tx_last_cmd);
}

void MainLoop::Update()
{
    test_led.Handle();
    ir_rx.Handle();  // IR 接收超时保护
    static float last_report_time = 0.0f;
    cnt++;

    if (DWT_GetTimeline_Sec() - last_report_time >= 4.0f)
    {
        last_report_time = DWT_GetTimeline_Sec();
        debug_cnt = MOD::host_pc.GetRecentRxFreqHz();
        APP::monit.LogInfo("[HostPC] RX_freq: %.1f Hz, cnt:%d", MOD::host_pc.GetRecentRxFreqHz(), cnt);
    }
}
