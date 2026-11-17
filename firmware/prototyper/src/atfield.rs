// SPDX-License-Identifier: MulanPSL-2.0 OR MIT

//! 调用外部 TSM（AT Field）的钩子。
//!
//! AT Field 是**独立镜像**，装载在机密半区的**固定物理地址**上，由固件在自己的
//! 初始化流程结束、下一阶段开始之前逐 hart 调用一次。它自己保存调用者上下文、
//! 切到自己的栈与 gp，运行完原样返回（见 AT Field 仓库的 `src/start.S`）。
//!
//! 因此这里可以把它当普通函数调用：`jalr` 过去，等它 `ret` 回来，继续固件的流程。

/// AT Field 入口的物理地址。
///
/// 4 GiB QEMU `virt` 的 RAM 是 `0x8000_0000..0x1_8000_0000`，取后半段起点作为
/// 机密内存与 TSM 的装载地址。**必须与 AT Field 的 `linker.ld` 保持一致**。
const ATFIELD_ENTRY: usize = 0x1_0000_0000;

/// `a1` 的角色取值，必须与 AT Field 的 `src/start.S` 一致。
const ROLE_PRIMARY: usize = 0;
const ROLE_SECONDARY: usize = 1;

/// 交给 AT Field 的下一阶段 FDT 地址（`a2`）。
///
/// 传的是**下一阶段那份**——固件已经打好补丁、准备交给 S 模式的那份（带
/// `/reserved-memory` 与固件镜像预留），不是 SPL 交来的原件。AT Field 要从里面
/// 读机密区边界，所以只能认这一份。
///
/// 存成全局而不是参数：AT Field 每个 hart 都要拿同一个值，而 secondary hart
/// 的 `BootInfo` 里没有这个运行期地址（它只由 boot hart 在 `init_board()` 后
/// 得到）。由 boot hart `store`、secondary 在 `PRIMARY_DONE` 之后 `load`，
/// 靠 `PRIMARY_DONE` 的 release/acquire 建立 happens-before。
static NEXT_STAGE_FDT: core::sync::atomic::AtomicUsize = core::sync::atomic::AtomicUsize::new(0);

/// 由 boot hart 在 `init_board()` 之后调用（见 `main.rs` 的 `boot_hart`）。
pub(crate) fn set_next_stage_fdt(address: usize) {
    NEXT_STAGE_FDT.store(address, core::sync::atomic::Ordering::Release);
}

/// boot hart 已完成 AT Field 的全局初始化。
///
/// 顺序契约：**secondary hart 必须等 boot hart 的 primary 初始化完成后再进来**。
/// 两个理由：
///   1. 语义：per-hart 初始化不应跑在全局初始化之前。
///   2. 现实：两边共用同一个 16550，而 THR 是单寄存器——boot hart 还在打固件
///      日志时，secondary hart 的字节会被互相覆盖（"等 LSR 为空再写"这一步
///      对两个写者不是原子的），实测会丢字节。
static PRIMARY_DONE: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

/// secondary hart 的串行锁：包住"调用 AT Field + 打返回日志"整段。
///
/// 串口是两台"独立软件"（固件与 TSM）共用的单一 16550，THR 只有一个寄存器；
/// 两个 hart 同时"等 LSR 为空再写"会互相吞字节（实测可见丢前缀、字符交错）。
/// 固件与 TSM 之间的锁 ABI 还没设计，所以在这里退一步：**同一时刻只让一个
/// secondary hart 进入这段区域**。boot hart 不需要（它跑完才放 secondary 进来）。
static SECONDARY_SERIAL: spin::Mutex<()> = spin::Mutex::new(());

/// 调用 AT Field 的每 hart 初始化入口。
///
/// 调用约定：`a0` = 硬件 hartid，`a1` = 角色（0 = boot hart，1 = 其它），
/// `a2` = 下一阶段 FDT 物理地址（0 表示未知）；
/// 返回后不破坏调用者保存的寄存器（AT Field 会恢复 `ra`/`sp`/`gp`/`s0`），
/// 因此可以直接继续执行。
///
/// 角色必须由固件告诉 AT Field：boot hart 是抢锁选举出来的，不一定 hartid 0，
/// AT Field 无法自行推断。
pub(crate) fn hart_init(is_boot_hart: bool) {
    if !is_boot_hart {
        // 等 boot hart 完成全局初始化（见 `PRIMARY_DONE` 的说明）。
        // 此刻机器中断是关的，自旋安全。
        while !PRIMARY_DONE.load(core::sync::atomic::Ordering::Acquire) {
            core::hint::spin_loop();
        }
    }

    // secondary hart 串行进入，避免与固件日志争抢同一个串口（见上）。
    let _serial = if is_boot_hart {
        None
    } else {
        Some(SECONDARY_SERIAL.lock())
    };

    let hart_id = riscv::register::mhartid::read();
    // SAFETY: 该地址由 AT Field 的链接脚本固定为镜像入口，镜像由 QEMU 的
    // loader（`-device loader,file=...,addr=0x100000000`）预先装入。
    // 入口遵循 C ABI 并且会正常返回。
    let entry: extern "C" fn(usize, usize, usize) = unsafe { core::mem::transmute(ATFIELD_ENTRY) };
    let role = if is_boot_hart {
        ROLE_PRIMARY
    } else {
        ROLE_SECONDARY
    };
    // Acquire 与 boot hart 的 Release 配对（这里也可能读到 boot hart 自己写的值）。
    let fdt = NEXT_STAGE_FDT.load(core::sync::atomic::Ordering::Acquire);
    entry(hart_id, role, fdt);

    // 调试用：先把这行日志（共用同一个 16550）打完，再放 secondary hart 进来，
    // 否则 boot hart 的日志会和 secondary 的首次打印抢 THR（会丢字节）。
    // 整个 hook 稳定后这行可以删。
    log::info!("AT Field returned control to the firmware");

    if is_boot_hart {
        PRIMARY_DONE.store(true, core::sync::atomic::Ordering::Release);
    }
}
