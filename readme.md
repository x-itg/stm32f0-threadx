# PendVC_Handle 
- 1、stm32cubemx nvic 去掉Pendablerequest for system service
- NVIC -> Code generation
- 利用stm32cubemx生成iar源代码

# 配置IAR工程  
- 2、将threadx源码的common和port拷贝到工作目录下面
- - iar编译中添加inc[C/C++ Complier  Preprocessor]中添加包含文件夹
- - - $PROJ_DIR$/../common/inc
- - - $PROJ_DIR$/../ports/cortex_m0/iar/inc
- - 添加源文件
- - - 添加common/src下面所有.c文件
- - - 添加port/cortex_m0/iar/example_build/tx_iar.c
- - - 添加port/cortex_m0/iar/example_build/tx_initialize_low_level.s
- - - 添加port/cortex_m0/iar/src/tx_thread_context_restore.s
- - - 添加port/cortex_m0/iar/src/tx_thread_context_save.s
- - - 添加port/cortex_m0/iar/src/tx_thread_interrupt_control.s
- - - 添加port/cortex_m0/iar/src/tx_thread_interrupt_disable.s
- - - 添加port/cortex_m0/iar/src/tx_thread_interrupt_restore.s
- - - 添加port/cortex_m0/iar/src/tx_thread_schedule.s
- - - 添加port/cortex_m0/iar/src/tx_thread_stack_build.s
- - - 添加port/cortex_m0/iar/src/tx_thread_system_return.s
- - - 添加port/cortex_m0/iar/src/tx_timer_interrupt.s

# 修改频率
- 3、tx_initalize_low_level.s中修改频率
- - - SYSTEM_CLOCK      EQU   48000000
- - - SYSTICK_CYCLES    EQU   ((SYSTEM_CLOCK / 1000) -1)



