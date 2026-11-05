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

/// 调用 AT Field 的每 hart 初始化入口。
///
/// 调用约定：`a0` = 硬件 hartid；返回后不破坏调用者保存的寄存器
/// （AT Field 会恢复 `ra`/`sp`/`gp`/`s0`），因此可以直接继续执行。
pub(crate) fn hart_init() {
    let hart_id = riscv::register::mhartid::read();
    // SAFETY: 该地址由 AT Field 的链接脚本固定为镜像入口，镜像由 QEMU 的
    // loader（`-device loader,file=...,addr=0x100000000`）预先装入。
    // 入口遵循 C ABI 并且会正常返回。
    let entry: extern "C" fn(usize) = unsafe { core::mem::transmute(ATFIELD_ENTRY) };
    entry(hart_id);
}
