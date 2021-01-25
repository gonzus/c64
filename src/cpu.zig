const std = @import("std");
const Type = @import("types.zig").Type;
const Status = @import("status.zig").Status;
const Memory = @import("memory.zig").Memory;
const testing = std.testing;

const allocator = std.heap.page_allocator;

pub const CPU = struct {
    const INITIAL_ADDRESS = 0xF000;
    const STACK_BASE = 0x0100;

    PC: Type.Word, // Program Counter
    PS: Status, // Processor Status

    regs: [4]Type.Byte,
    memory: Memory, // Memory bank with 64 KB -- wow

    ticks: u32, // Cycle counter

    // TODO: I would like these to be members of an enum, and have an array of
    // them.
    const A = 0;
    const X = 1;
    const Y = 2;
    const SP = 3;

    const BitOp = enum {
        And,
        InclusiveOr,
        ExclusiveOr,
        Bit,
    };

    const AddressingMode = enum {
        Immediate,
        ZeroPage,
        ZeroPageX,
        ZeroPageY,
        Absolute,
        AbsoluteX,
        AbsoluteY,
        IndirectX,
        IndirectY,
    };

    const OP = enum(Type.Byte) {
        LDA_IMM = 0xA9,
        LDA_ZP = 0xA5,
        LDA_ZPX = 0xB5,
        LDA_ABS = 0xAD,
        LDA_ABSX = 0xBD,
        LDA_ABSY = 0xB9,
        LDA_INDX = 0xA1,
        LDA_INDY = 0xB1,

        LDX_IMM = 0xA2,
        LDX_ZP = 0xA6,
        LDX_ZPY = 0xB6,
        LDX_ABS = 0xAE,
        LDX_ABSY = 0xBE,

        LDY_IMM = 0xA0,
        LDY_ZP = 0xA4,
        LDY_ZPX = 0xB4,
        LDY_ABS = 0xAC,
        LDY_ABSX = 0xBC,

        STA_ZP = 0x85,
        STA_ZPX = 0x95,
        STA_ABS = 0x8D,
        STA_ABSX = 0x9D,
        STA_ABSY = 0x99,
        STA_INDX = 0x81,
        STA_INDY = 0x91,

        STX_ZP = 0x86,
        STX_ZPY = 0x96,
        STX_ABS = 0x8E,

        STY_ZP = 0x84,
        STY_ZPX = 0x94,
        STY_ABS = 0x8C,

        TAX = 0xAA,
        TAY = 0xA8,
        TXA = 0x8A,
        TYA = 0x98,
        TSX = 0xBA,
        TXS = 0x9A,

        PHA = 0x48,
        PHP = 0x08,
        PLA = 0x68,
        PLP = 0x28,

        AND_IMM = 0x29,
        AND_ZP = 0x25,
        AND_ZPX = 0x35,
        AND_ABS = 0x2D,
        AND_ABSX = 0x3D,
        AND_ABSY = 0x39,
        AND_INDX = 0x21,
        AND_INDY = 0x31,

        EOR_IMM = 0x49,
        EOR_ZP = 0x45,
        EOR_ZPX = 0x55,
        EOR_ABS = 0x4D,
        EOR_ABSX = 0x5D,
        EOR_ABSY = 0x59,
        EOR_INDX = 0x41,
        EOR_INDY = 0x51,

        ORA_IMM = 0x09,
        ORA_ZP = 0x05,
        ORA_ZPX = 0x15,
        ORA_ABS = 0x0D,
        ORA_ABSX = 0x1D,
        ORA_ABSY = 0x19,
        ORA_INDX = 0x01,
        ORA_INDY = 0x11,

        BIT_ZP = 0x24,
        BIT_ABS = 0x2C,

        NOP = 0xEA,
    };

    pub fn init() CPU {
        var self = CPU{
            .PC = undefined,
            .PS = undefined,
            .regs = undefined,
            .memory = undefined,
            .ticks = undefined,
        };
        self.reset(INITIAL_ADDRESS);
        return self;
    }

    pub fn reset(self: *CPU, address: Type.Word) void {
        self.PC = address;
        self.PS.clear();
        self.regs[SP] = 0xFF;
        self.regs[A] = 0;
        self.regs[X] = 0;
        self.regs[Y] = 0;
        self.memory.clear();
        self.ticks = 0;
    }

    pub fn run(self: *CPU, limit: u32) u32 {
        const start = self.ticks;
        while ((self.ticks - start) < limit) {
            const op = @intToEnum(OP, self.readByte(self.PC));
            self.PC += 1;
            switch (op) {
                OP.LDA_IMM => self.fetch(A, .Immediate),
                OP.LDA_ZP => self.fetch(A, .ZeroPage),
                OP.LDA_ZPX => self.fetch(A, .ZeroPageX),
                OP.LDA_ABS => self.fetch(A, .Absolute),
                OP.LDA_ABSX => self.fetch(A, .AbsoluteX),
                OP.LDA_ABSY => self.fetch(A, .AbsoluteY),
                OP.LDA_INDX => self.fetch(A, .IndirectX),
                OP.LDA_INDY => self.fetch(A, .IndirectY),

                OP.LDX_IMM => self.fetch(X, .Immediate),
                OP.LDX_ZP => self.fetch(X, .ZeroPage),
                OP.LDX_ZPY => self.fetch(X, .ZeroPageY),
                OP.LDX_ABS => self.fetch(X, .Absolute),
                OP.LDX_ABSY => self.fetch(X, .AbsoluteY),

                OP.LDY_IMM => self.fetch(Y, .Immediate),
                OP.LDY_ZP => self.fetch(Y, .ZeroPage),
                OP.LDY_ZPX => self.fetch(Y, .ZeroPageX),
                OP.LDY_ABS => self.fetch(Y, .Absolute),
                OP.LDY_ABSX => self.fetch(Y, .AbsoluteX),

                OP.STA_ZP => self.store(A, .ZeroPage),
                OP.STA_ZPX => self.store(A, .ZeroPageX),
                OP.STA_ABS => self.store(A, .Absolute),
                OP.STA_ABSX => self.store(A, .AbsoluteX),
                OP.STA_ABSY => self.store(A, .AbsoluteY),
                OP.STA_INDX => self.store(A, .IndirectX),
                OP.STA_INDY => self.store(A, .IndirectY),

                OP.STX_ZP => self.store(X, .ZeroPage),
                OP.STX_ZPY => self.store(X, .ZeroPageY),
                OP.STX_ABS => self.store(X, .Absolute),

                OP.STY_ZP => self.store(Y, .ZeroPage),
                OP.STY_ZPX => self.store(Y, .ZeroPageX),
                OP.STY_ABS => self.store(Y, .Absolute),

                OP.TAX => self.transfer(A, X, true),
                OP.TAY => self.transfer(A, Y, true),
                OP.TXA => self.transfer(X, A, true),
                OP.TYA => self.transfer(Y, A, true),
                OP.TSX => self.transfer(SP, X, true),
                OP.TXS => self.transfer(X, SP, false),

                OP.PHA => self.push(self.regs[A]),
                OP.PHP => self.push(self.PS.byte),
                OP.PLA => self.regs[A] = self.pop(true),
                OP.PLP => self.PS.byte = self.pop(false),

                OP.AND_IMM => self.bitOp(.And, A, .Immediate),
                OP.AND_ZP => self.bitOp(.And, A, .ZeroPage),
                OP.AND_ZPX => self.bitOp(.And, A, .ZeroPageX),
                OP.AND_ABS => self.bitOp(.And, A, .Absolute),
                OP.AND_ABSX => self.bitOp(.And, A, .AbsoluteX),
                OP.AND_ABSY => self.bitOp(.And, A, .AbsoluteY),
                OP.AND_INDX => self.bitOp(.And, A, .IndirectX),
                OP.AND_INDY => self.bitOp(.And, A, .IndirectY),

                OP.EOR_IMM => self.bitOp(.ExclusiveOr, A, .Immediate),
                OP.EOR_ZP => self.bitOp(.ExclusiveOr, A, .ZeroPage),
                OP.EOR_ZPX => self.bitOp(.ExclusiveOr, A, .ZeroPageX),
                OP.EOR_ABS => self.bitOp(.ExclusiveOr, A, .Absolute),
                OP.EOR_ABSX => self.bitOp(.ExclusiveOr, A, .AbsoluteX),
                OP.EOR_ABSY => self.bitOp(.ExclusiveOr, A, .AbsoluteY),
                OP.EOR_INDX => self.bitOp(.ExclusiveOr, A, .IndirectX),
                OP.EOR_INDY => self.bitOp(.ExclusiveOr, A, .IndirectY),

                OP.ORA_IMM => self.bitOp(.InclusiveOr, A, .Immediate),
                OP.ORA_ZP => self.bitOp(.InclusiveOr, A, .ZeroPage),
                OP.ORA_ZPX => self.bitOp(.InclusiveOr, A, .ZeroPageX),
                OP.ORA_ABS => self.bitOp(.InclusiveOr, A, .Absolute),
                OP.ORA_ABSX => self.bitOp(.InclusiveOr, A, .AbsoluteX),
                OP.ORA_ABSY => self.bitOp(.InclusiveOr, A, .AbsoluteY),
                OP.ORA_INDX => self.bitOp(.InclusiveOr, A, .IndirectX),
                OP.ORA_INDY => self.bitOp(.InclusiveOr, A, .IndirectY),

                OP.BIT_ZP => self.bitOp(.Bit, A, .ZeroPage),
                OP.BIT_ABS => self.bitOp(.Bit, A, .Absolute),

                OP.NOP => self.tick(),
            }
        }
        return self.ticks - start;
    }

    fn fetch(self: *CPU, register: usize, mode: AddressingMode) void {
        const address = self.computeAddress(mode, false);
        const value = self.readByte(address);
        self.regs[register] = value;
        self.setNZ(self.regs[register]);
    }

    fn store(self: *CPU, register: usize, mode: AddressingMode) void {
        const address = self.computeAddress(mode, false);
        self.writeByte(address, self.regs[register]);
    }

    fn transfer(self: *CPU, source: usize, target: usize, flags: bool) void {
        self.regs[target] = self.regs[source];
        if (flags) {
            self.setNZ(self.regs[target]);
        }
        self.tick();
    }

    fn push(self: *CPU, value: Type.Byte) void {
        const address = @as(Type.Word, STACK_BASE) + self.regs[SP];
        self.writeByte(address, value);
        self.regs[SP] -%= 1;
        self.tick();
    }

    fn pop(self: *CPU, flags: bool) Type.Byte {
        self.regs[SP] +%= 1;
        self.tick();
        const address = @as(Type.Word, STACK_BASE) + self.regs[SP];
        self.tick();
        const value = self.readByte(address);
        if (flags) {
            self.setNZ(value);
        }
        return value;
    }

    fn bitOp(self: *CPU, op: BitOp, register: usize, mode: AddressingMode) void {
        const address = self.computeAddress(mode, false);
        const value = self.readByte(address);
        const result = switch (op) {
            .And, .Bit => self.regs[register] & value,
            .InclusiveOr => self.regs[register] | value,
            .ExclusiveOr => self.regs[register] ^ value,
        };
        self.setNZ(result);
        if (op == .Bit) {
            self.PS.bits.V = if ((result & 0b01000000) > 0) 1 else 0;
        } else {
            self.regs[register] = result;
        }
    }

    fn computeAddress(self: *CPU, mode: AddressingMode, alwaysUseExtra: bool) Type.Word {
        const address = switch (mode) {
            .Immediate => blk: {
                const address = self.PC;
                self.PC += 1;
                break :blk address;
            },
            .ZeroPage => blk: {
                const address = @as(Type.Word, self.readByte(self.PC));
                self.PC += 1;
                break :blk address;
            },
            .ZeroPageX => blk: {
                var address = @as(Type.Word, self.readByte(self.PC));
                self.PC += 1;
                address +%= self.regs[X];
                self.tick();
                break :blk address;
            },
            .ZeroPageY => blk: {
                var address = @as(Type.Word, self.readByte(self.PC));
                self.PC += 1;
                address +%= self.regs[Y];
                self.tick();
                break :blk address;
            },
            .Absolute => blk: {
                const address = self.readWord(self.PC);
                self.PC += 2;
                break :blk address;
            },
            .AbsoluteX => blk: {
                const initial = self.readWord(self.PC);
                const final = initial + self.regs[X];
                if (alwaysUseExtra or !samePage(initial, final)) {
                    self.tick();
                }
                break :blk final;
            },
            .AbsoluteY => blk: {
                const initial = self.readWord(self.PC);
                const final = initial + self.regs[Y];
                if (alwaysUseExtra or !samePage(initial, final)) {
                    self.tick();
                }
                break :blk final;
            },
            .IndirectX => blk: {
                var address = @as(Type.Word, self.readByte(self.PC));
                self.PC += 1;
                address +%= self.regs[X];
                self.tick();
                const final = self.readWord(address);
                break :blk final;
            },
            .IndirectY => blk: {
                const address = @as(Type.Word, self.readByte(self.PC));
                self.PC += 1;
                const initial = self.readWord(address);
                const final = initial + self.regs[Y];
                if (alwaysUseExtra or !samePage(initial, final)) {
                    self.tick();
                }
                break :blk final;
            },
        };
        return address;
    }

    fn readByte(self: *CPU, address: Type.Word) Type.Byte {
        const value = self.memory.data[address];
        self.tick();
        return value;
    }

    fn readWord(self: *CPU, address: Type.Word) Type.Word {
        const lo = @as(Type.Word, self.readByte(address + 0));
        const hi = @as(Type.Word, self.readByte(address + 1)) << 8;
        const value = hi | lo;
        return value;
    }

    fn writeByte(self: *CPU, address: Type.Word, value: Type.Byte) void {
        self.memory.data[address] = value;
        self.tick();
    }

    fn tick(self: *CPU) void {
        self.ticks += 1;
    }

    fn setNZ(self: *CPU, value: Type.Word) void {
        self.PS.bits.N = if ((value & 0b10000000) > 0) 1 else 0;
        self.PS.bits.Z = if ((value | 0b00000000) > 0) 0 else 1;
    }

    fn samePage(p1: Type.Word, p2: Type.Word) bool {
        // Sometimes adding something to an address will incur in an extra tick
        // ONLY when that caused the address to cross onto another page.
        return (p1 & 0xFF00) == (p2 & 0xFF00);
    }
};

// =========================================================

const TEST_ADDRESS = 0x4433;

test "create CPU" {
    var cpu = CPU.init();
    testing.expect(cpu.PC == CPU.INITIAL_ADDRESS);
    cpu.reset(TEST_ADDRESS);
    testing.expect(cpu.PC == TEST_ADDRESS);
}

fn test_load_register(cpu: *CPU, register: usize, address: Type.Word, ticks: u32) void {
    const Data = struct {
        v: Type.Byte,
        N: Type.Bit,
        Z: Type.Bit,
    };
    const data = [_]Data{
        .{ .v = 0x11, .N = 0, .Z = 0 },
        .{ .v = 0xF0, .N = 1, .Z = 0 },
        .{ .v = 0x00, .N = 0, .Z = 1 },
    };
    for (data) |d| {
        cpu.PC = TEST_ADDRESS;
        cpu.memory.data[address] = d.v; // put value in memory address
        const prevPS = cpu.PS; // remember PS
        const prevRegs = cpu.regs; // remember registers
        cpu.regs[register] = 0; // set desired register to 0
        cpu.PS.byte = 0; // set PS to 0
        const used = cpu.run(ticks);

        testing.expect(used == ticks);
        testing.expect(cpu.regs[register] == d.v); // got correct value in register?
        testing.expect(cpu.PS.bits.N == d.N); // got correct N bit?
        testing.expect(cpu.PS.bits.Z == d.Z); // got correct Z bit?

        // other bits didn't change?
        testing.expect(cpu.PS.bits.C == prevPS.bits.C);
        testing.expect(cpu.PS.bits.I == prevPS.bits.I);
        testing.expect(cpu.PS.bits.D == prevPS.bits.D);
        testing.expect(cpu.PS.bits.B == prevPS.bits.B);
        testing.expect(cpu.PS.bits.V == prevPS.bits.V);

        // registers either got set or didn't change?
        testing.expect(register == CPU.A or cpu.regs[CPU.A] == prevRegs[CPU.A]);
        testing.expect(register == CPU.X or cpu.regs[CPU.X] == prevRegs[CPU.X]);
        testing.expect(register == CPU.Y or cpu.regs[CPU.Y] == prevRegs[CPU.Y]);
        testing.expect(register == CPU.SP or cpu.regs[CPU.SP] == prevRegs[CPU.SP]);
    }
}

fn test_save_register(cpu: *CPU, register: usize, address: Type.Word, ticks: u32) void {
    const Data = struct {
        v: Type.Byte,
    };
    const data = [_]Data{
        .{ .v = 0x11 },
        .{ .v = 0xF0 },
        .{ .v = 0x00 },
    };
    for (data) |d| {
        cpu.PC = TEST_ADDRESS;
        cpu.regs[register] = d.v; // put value in register
        const prevPS = cpu.PS; // remember PS
        const prevRegs = cpu.regs; // remember registers
        cpu.memory.data[address] = 0; // set desired address to 0
        cpu.PS.byte = 0; // set PS to 0
        const used = cpu.run(ticks);

        testing.expect(used == ticks);
        testing.expect(cpu.memory.data[address] == d.v); // got correct value in memory?
        testing.expect(cpu.PS.byte == prevPS.byte); // PS didn't change?

        // registers didn't change?
        testing.expect(cpu.regs[CPU.A] == prevRegs[CPU.A]);
        testing.expect(cpu.regs[CPU.X] == prevRegs[CPU.X]);
        testing.expect(cpu.regs[CPU.Y] == prevRegs[CPU.Y]);
        testing.expect(cpu.regs[CPU.SP] == prevRegs[CPU.SP]);
    }
}

fn test_transfer_register(cpu: *CPU, source: usize, target: usize, flags: bool, ticks: u32) void {
    const Data = struct {
        v: Type.Byte,
        N: Type.Bit,
        Z: Type.Bit,
    };
    const data = [_]Data{
        .{ .v = 0x11, .N = 0, .Z = 0 },
        .{ .v = 0xF0, .N = 1, .Z = 0 },
        .{ .v = 0x00, .N = 0, .Z = 1 },
    };
    for (data) |d| {
        cpu.PC = TEST_ADDRESS;
        cpu.regs[source] = d.v; // put value in source register
        const prevPS = cpu.PS; // remember PS
        const prevRegs = cpu.regs; // remember registers
        cpu.regs[target] = 0; // set target register to 0
        cpu.PS.byte = 0; // set PS to 0
        const used = cpu.run(ticks);

        testing.expect(used == ticks);
        testing.expect(cpu.regs[target] == d.v); // got correct value in target registry?
        if (flags) {
            testing.expect(cpu.PS.bits.N == d.N); // got correct N bit?
            testing.expect(cpu.PS.bits.Z == d.Z); // got correct Z bit?
        } else {
            testing.expect(cpu.PS.bits.N == prevPS.bits.N);
            testing.expect(cpu.PS.bits.Z == prevPS.bits.Z);
        }

        // other bits didn't change?
        testing.expect(cpu.PS.bits.C == prevPS.bits.C);
        testing.expect(cpu.PS.bits.I == prevPS.bits.I);
        testing.expect(cpu.PS.bits.D == prevPS.bits.D);
        testing.expect(cpu.PS.bits.B == prevPS.bits.B);
        testing.expect(cpu.PS.bits.V == prevPS.bits.V);

        // registers either got set or didn't change?
        testing.expect(target == CPU.A or cpu.regs[CPU.A] == prevRegs[CPU.A]);
        testing.expect(target == CPU.X or cpu.regs[CPU.X] == prevRegs[CPU.X]);
        testing.expect(target == CPU.Y or cpu.regs[CPU.Y] == prevRegs[CPU.Y]);
        testing.expect(target == CPU.SP or cpu.regs[CPU.SP] == prevRegs[CPU.SP]);
    }
}

fn test_push_register(cpu: *CPU, location: *Type.Byte, ticks: u32) void {
    const Data = struct {
        v: Type.Byte,
    };
    const data = [_]Data{
        .{ .v = 0x11 },
        .{ .v = 0xF0 },
        .{ .v = 0x00 },
    };
    for (data) |d| {
        cpu.PC = TEST_ADDRESS;
        cpu.PS.byte = 0; // set PS to 0
        location.* = d.v; // put value in register
        const prevPS = cpu.PS; // remember PS
        const prevRegs = cpu.regs; // remember registers
        const used = cpu.run(ticks);

        testing.expect(used == ticks);
        testing.expect(cpu.regs[CPU.SP] == prevRegs[CPU.SP] - 1); // did SP move?
        const address = @as(Type.Word, CPU.STACK_BASE) + prevRegs[CPU.SP];
        testing.expect(cpu.memory.data[address] == d.v); // did the value get pushed?

        // none of the bits changed?
        testing.expect(cpu.PS.byte == prevPS.byte);

        // registers didn't change?
        testing.expect(cpu.regs[CPU.A] == prevRegs[CPU.A]);
        testing.expect(cpu.regs[CPU.X] == prevRegs[CPU.X]);
        testing.expect(cpu.regs[CPU.Y] == prevRegs[CPU.Y]);
    }
}

fn test_pop_register(cpu: *CPU, location: *Type.Byte, ticks: u32) void {
    const Data = struct {
        v: Type.Byte,
        N: Type.Bit,
        Z: Type.Bit,
    };
    const data = [_]Data{
        .{ .v = 0x11, .N = 0, .Z = 0 },
        .{ .v = 0xF0, .N = 1, .Z = 0 },
        .{ .v = 0x00, .N = 0, .Z = 1 },
    };
    var SP: Type.Byte = 0xFF;
    for (data) |d| {
        const address = @as(Type.Word, CPU.STACK_BASE) + SP;
        cpu.memory.data[address] = d.v; // set value in stack
        SP -%= 1;
    }
    cpu.regs[CPU.SP] = SP;
    for (data) |d, p| {
        const pos = data.len - 1 - p;
        cpu.PC = TEST_ADDRESS;
        cpu.PS.byte = 0; // set PS to 0
        location.* = 0;
        const prevPS = cpu.PS; // remember PS
        const prevRegs = cpu.regs; // remember registers
        const used = cpu.run(ticks);

        testing.expect(used == ticks);
        testing.expect(cpu.regs[CPU.SP] == prevRegs[CPU.SP] + 1); // did SP move?
        const address = @as(Type.Word, CPU.STACK_BASE) + prevRegs[CPU.SP];
        testing.expect(location.* == data[pos].v); // did the value get popped?

        if (location == &cpu.regs[CPU.A]) {
            testing.expect(cpu.PS.bits.N == data[pos].N); // got correct N bit?
            testing.expect(cpu.PS.bits.Z == data[pos].Z); // got correct Z bit?
            // other bits didn't change?
            testing.expect(cpu.PS.bits.C == prevPS.bits.C);
            testing.expect(cpu.PS.bits.I == prevPS.bits.I);
            testing.expect(cpu.PS.bits.D == prevPS.bits.D);
            testing.expect(cpu.PS.bits.B == prevPS.bits.B);
            testing.expect(cpu.PS.bits.V == prevPS.bits.V);
        }

        // registers didn't change?
        testing.expect(location == &cpu.regs[CPU.A] or cpu.regs[CPU.A] == prevRegs[CPU.A]);
        testing.expect(cpu.regs[CPU.X] == prevRegs[CPU.X]);
        testing.expect(cpu.regs[CPU.Y] == prevRegs[CPU.Y]);
    }
    testing.expect(cpu.regs[CPU.SP] == 0xFF);
}

fn test_bitop_register(cpu: *CPU, op: CPU.BitOp, register: usize, address: Type.Word, ticks: u32) void {
    const Data = struct {
        m: Type.Byte,
        r: Type.Byte,
    };
    const data = [_]Data{
        .{
            .m = 0b00110001,
            .r = 0b10101010,
        },
        .{
            .m = 0b01010101,
            .r = 0b10101010,
        },
        .{
            .m = 0b10110011,
            .r = 0b10101010,
        },
    };
    for (data) |d| {
        cpu.PC = TEST_ADDRESS;
        cpu.memory.data[address] = d.m; // put value in memory address
        const prevPS = cpu.PS; // remember PS
        const prevRegs = cpu.regs; // remember registers
        cpu.regs[register] = d.r; // set desired register
        cpu.PS.byte = 0; // set PS to 0

        const afterR: Type.Byte = switch (op) {
            .And, .Bit => d.m & d.r,
            .InclusiveOr => d.m | d.r,
            .ExclusiveOr => d.m ^ d.r,
        };
        const afterN: Type.Bit = if ((afterR & 0b10000000) > 0) 1 else 0;
        const afterV: Type.Bit = if ((afterR & 0b01000000) > 0) 1 else 0;
        const afterZ: Type.Bit = if ((afterR | 0b00000000) > 0) 0 else 1;

        const used = cpu.run(ticks);
        testing.expect(used == ticks);

        if (op == .Bit) {
            testing.expect(cpu.PS.bits.V == afterV); // got correct N bit?
        } else {
            testing.expect(cpu.regs[register] == afterR); // got correct after value in register?
        }
        testing.expect(cpu.PS.bits.N == afterN); // got correct N bit?
        testing.expect(cpu.PS.bits.Z == afterZ); // got correct Z bit?

        // other bits didn't change?
        testing.expect(cpu.PS.bits.C == prevPS.bits.C);
        testing.expect(cpu.PS.bits.I == prevPS.bits.I);
        testing.expect(cpu.PS.bits.D == prevPS.bits.D);
        testing.expect(cpu.PS.bits.B == prevPS.bits.B);
        testing.expect(cpu.PS.bits.V == prevPS.bits.V);

        // registers either got set or didn't change?
        testing.expect(register == CPU.A or cpu.regs[CPU.A] == prevRegs[CPU.A]);
        testing.expect(register == CPU.X or cpu.regs[CPU.X] == prevRegs[CPU.X]);
        testing.expect(register == CPU.Y or cpu.regs[CPU.Y] == prevRegs[CPU.Y]);
        testing.expect(register == CPU.SP or cpu.regs[CPU.SP] == prevRegs[CPU.SP]);
    }
}

// LDA tests

test "run LDA_IMM" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0xA9;
    test_load_register(&cpu, CPU.A, TEST_ADDRESS + 1, 2);
}

test "run LDA_ZP" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0xA5;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    test_load_register(&cpu, CPU.A, 0x0011, 3);
}

test "run LDA_ZPX" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.X] = 7;
    cpu.memory.data[TEST_ADDRESS + 0] = 0xB5;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    test_load_register(&cpu, CPU.A, 0x0011 + 7, 4);
}

test "run LDA_ABS" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0xAD;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_load_register(&cpu, CPU.A, 0x8311, 4);
}

test "run LDA_ABSX same page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.X] = 7;
    cpu.memory.data[TEST_ADDRESS + 0] = 0xBD;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_load_register(&cpu, CPU.A, 0x8311 + 7, 4);
}

test "run LDA_ABSY same page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.Y] = 7;
    cpu.memory.data[TEST_ADDRESS + 0] = 0xB9;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_load_register(&cpu, CPU.A, 0x8311 + 7, 4);
}

test "run LDA_ABSX cross page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.X] = 0xFE;
    cpu.memory.data[TEST_ADDRESS + 0] = 0xBD;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_load_register(&cpu, CPU.A, 0x8311 + 0xFE, 5);
}

test "run LDA_ABSY cross page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.Y] = 0xFE;
    cpu.memory.data[TEST_ADDRESS + 0] = 0xB9;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_load_register(&cpu, CPU.A, 0x8311 + 0xFE, 5);
}

test "run LDA_INDX" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.X] = 4;
    cpu.memory.data[TEST_ADDRESS + 0] = 0xA1;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x20;
    cpu.memory.data[0x20 + 4 + 0] = 0x74;
    cpu.memory.data[0x20 + 4 + 1] = 0x20;
    test_load_register(&cpu, CPU.A, 0x2074, 6);
}

test "run LDA_INDY same page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.Y] = 0x10;
    cpu.memory.data[TEST_ADDRESS + 0] = 0xB1;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x86;
    cpu.memory.data[0x86 + 0] = 0x28;
    cpu.memory.data[0x86 + 1] = 0x40;
    test_load_register(&cpu, CPU.A, 0x4028 + 0x10, 5);
}

test "run LDA_INDY cross page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.Y] = 0xFE;
    cpu.memory.data[TEST_ADDRESS + 0] = 0xB1;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x86;
    cpu.memory.data[0x86 + 0] = 0x28;
    cpu.memory.data[0x86 + 1] = 0x40;
    test_load_register(&cpu, CPU.A, 0x4028 + 0xFE, 6);
}

// LDX tests

test "run LDX_IMM" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0xA2;
    test_load_register(&cpu, CPU.X, TEST_ADDRESS + 1, 2);
}

test "run LDX_ZP" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0xA6;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    test_load_register(&cpu, CPU.X, 0x0011, 3);
}

test "run LDX_ZPY" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.Y] = 7;
    cpu.memory.data[TEST_ADDRESS + 0] = 0xB6;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    test_load_register(&cpu, CPU.X, 0x0011 + 7, 4);
}

test "run LDX_ABS" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0xAE;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_load_register(&cpu, CPU.X, 0x8311, 4);
}

test "run LDX_ABSY same page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.Y] = 7;
    cpu.memory.data[TEST_ADDRESS + 0] = 0xBE;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_load_register(&cpu, CPU.X, 0x8311 + 7, 4);
}

test "run LDX_ABSY cross page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.Y] = 0xFE;
    cpu.memory.data[TEST_ADDRESS + 0] = 0xBE;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_load_register(&cpu, CPU.X, 0x8311 + 0xFE, 5);
}

// LDY tests

test "run LDY_IMM" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0xA0;
    test_load_register(&cpu, CPU.Y, TEST_ADDRESS + 1, 2);
}

test "run LDY_ZP" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0xA4;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    test_load_register(&cpu, CPU.Y, 0x0011, 3);
}

test "run LDY_ZPX" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.X] = 7;
    cpu.memory.data[TEST_ADDRESS + 0] = 0xB4;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    test_load_register(&cpu, CPU.Y, 0x0011 + 7, 4);
}

test "run LDY_ABS" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0xAC;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_load_register(&cpu, CPU.Y, 0x8311, 4);
}

test "run LDY_ABSX same page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.X] = 7;
    cpu.memory.data[TEST_ADDRESS + 0] = 0xBC;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_load_register(&cpu, CPU.Y, 0x8311 + 7, 4);
}

test "run LDY_ABSX cross page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.X] = 0xFE;
    cpu.memory.data[TEST_ADDRESS + 0] = 0xBC;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_load_register(&cpu, CPU.Y, 0x8311 + 0xFE, 5);
}

// STA tests

test "run STA_ZP" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0x85;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    test_save_register(&cpu, CPU.A, 0x0011, 3);
}

test "run STA_ZPX" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.X] = 7;
    cpu.memory.data[TEST_ADDRESS + 0] = 0x95;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    test_save_register(&cpu, CPU.A, 0x0011 + 7, 4);
}

test "run STA_ABS" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0x8D;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_save_register(&cpu, CPU.A, 0x8311, 4);
}

test "run STA_ABSX same page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.X] = 7;
    cpu.memory.data[TEST_ADDRESS + 0] = 0x9D;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_save_register(&cpu, CPU.A, 0x8311 + 7, 4);
}

test "run STA_ABSY same page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.Y] = 7;
    cpu.memory.data[TEST_ADDRESS + 0] = 0x99;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_save_register(&cpu, CPU.A, 0x8311 + 7, 4);
}

test "run STA_ABSX cross page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.X] = 0xFE;
    cpu.memory.data[TEST_ADDRESS + 0] = 0x9D;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_save_register(&cpu, CPU.A, 0x8311 + 0xFE, 5);
}

test "run STA_ABSY cross page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.Y] = 0xFE;
    cpu.memory.data[TEST_ADDRESS + 0] = 0x99;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_save_register(&cpu, CPU.A, 0x8311 + 0xFE, 5);
}

test "run STA_INDX" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.X] = 4;
    cpu.memory.data[TEST_ADDRESS + 0] = 0x81;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x20;
    cpu.memory.data[0x20 + 4 + 0] = 0x74;
    cpu.memory.data[0x20 + 4 + 1] = 0x20;
    test_save_register(&cpu, CPU.A, 0x2074, 6);
}

test "run STA_INDY same page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.Y] = 0x10;
    cpu.memory.data[TEST_ADDRESS + 0] = 0x91;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x86;
    cpu.memory.data[0x86 + 0] = 0x28;
    cpu.memory.data[0x86 + 1] = 0x40;
    test_save_register(&cpu, CPU.A, 0x4028 + 0x10, 5);
}

test "run STA_INDY cross page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.Y] = 0xFE;
    cpu.memory.data[TEST_ADDRESS + 0] = 0x91;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x86;
    cpu.memory.data[0x86 + 0] = 0x28;
    cpu.memory.data[0x86 + 1] = 0x40;
    test_save_register(&cpu, CPU.A, 0x4028 + 0xFE, 6);
}

// STX tests

test "run STX_ZP" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0x86;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    test_save_register(&cpu, CPU.X, 0x0011, 3);
}

test "run STX_ZPY" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.Y] = 7;
    cpu.memory.data[TEST_ADDRESS + 0] = 0x96;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    test_save_register(&cpu, CPU.X, 0x0011 + 7, 4);
}

test "run STX_ABS" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0x8E;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_save_register(&cpu, CPU.X, 0x8311, 4);
}

// STY tests

test "run STY_ZP" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0x84;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    test_save_register(&cpu, CPU.Y, 0x0011, 3);
}

test "run STY_ZPX" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.X] = 7;
    cpu.memory.data[TEST_ADDRESS + 0] = 0x94;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    test_save_register(&cpu, CPU.Y, 0x0011 + 7, 4);
}

test "run STY_ABS" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0x8C;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_save_register(&cpu, CPU.Y, 0x8311, 4);
}

// TRANSFER tests

test "run TAX" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0xAA;
    test_transfer_register(&cpu, CPU.A, CPU.X, true, 2);
}

test "run TAY" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0xA8;
    test_transfer_register(&cpu, CPU.A, CPU.Y, true, 2);
}

test "run TXA" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0x8A;
    test_transfer_register(&cpu, CPU.X, CPU.A, true, 2);
}

test "run TYA" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0x98;
    test_transfer_register(&cpu, CPU.Y, CPU.A, true, 2);
}

test "run TSX" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0xBA;
    test_transfer_register(&cpu, CPU.SP, CPU.X, true, 2);
}

test "run TXS" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0x9A;
    test_transfer_register(&cpu, CPU.X, CPU.SP, false, 2);
}

// push

test "run PHA" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0x48;
    test_push_register(&cpu, &cpu.regs[CPU.A], 3);
}

test "run PHP" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0x08;
    test_push_register(&cpu, &cpu.PS.byte, 3);
}

// pop

test "run PLA" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0x68;
    test_pop_register(&cpu, &cpu.regs[CPU.A], 4);
}

test "run PLP" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0x28;
    test_pop_register(&cpu, &cpu.PS.byte, 4);
}

// AND tests

test "run AND_IMM" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0x29;
    test_bitop_register(&cpu, .And, CPU.A, TEST_ADDRESS + 1, 2);
}

test "run AND_ZP" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0x25;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    test_bitop_register(&cpu, .And, CPU.A, 0x0011, 3);
}

test "run AND_ZPX" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.X] = 7;
    cpu.memory.data[TEST_ADDRESS + 0] = 0x35;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    test_bitop_register(&cpu, .And, CPU.A, 0x0011 + 7, 4);
}

test "run AND_ABS" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0x2D;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_bitop_register(&cpu, .And, CPU.A, 0x8311, 4);
}

test "run AND_ABSX same page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.X] = 7;
    cpu.memory.data[TEST_ADDRESS + 0] = 0x3D;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_bitop_register(&cpu, .And, CPU.A, 0x8311 + 7, 4);
}

test "run AND_ABSY same page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.Y] = 7;
    cpu.memory.data[TEST_ADDRESS + 0] = 0x39;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_bitop_register(&cpu, .And, CPU.A, 0x8311 + 7, 4);
}

test "run AND_ABSX cross page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.X] = 0xFE;
    cpu.memory.data[TEST_ADDRESS + 0] = 0x3D;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_bitop_register(&cpu, .And, CPU.A, 0x8311 + 0xFE, 5);
}

test "run AND_ABSY cross page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.Y] = 0xFE;
    cpu.memory.data[TEST_ADDRESS + 0] = 0x39;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_bitop_register(&cpu, .And, CPU.A, 0x8311 + 0xFE, 5);
}

test "run AND_INDX" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.X] = 4;
    cpu.memory.data[TEST_ADDRESS + 0] = 0x21;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x20;
    cpu.memory.data[0x20 + 4 + 0] = 0x74;
    cpu.memory.data[0x20 + 4 + 1] = 0x20;
    test_bitop_register(&cpu, .And, CPU.A, 0x2074, 6);
}

test "run AND_INDY same page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.Y] = 0x10;
    cpu.memory.data[TEST_ADDRESS + 0] = 0x31;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x86;
    cpu.memory.data[0x86 + 0] = 0x28;
    cpu.memory.data[0x86 + 1] = 0x40;
    test_bitop_register(&cpu, .And, CPU.A, 0x4028 + 0x10, 5);
}

test "run AND_INDY cross page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.Y] = 0xFE;
    cpu.memory.data[TEST_ADDRESS + 0] = 0x31;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x86;
    cpu.memory.data[0x86 + 0] = 0x28;
    cpu.memory.data[0x86 + 1] = 0x40;
    test_bitop_register(&cpu, .And, CPU.A, 0x4028 + 0xFE, 6);
}

// EOR tests

test "run EOR_IMM" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0x49;
    test_bitop_register(&cpu, .ExclusiveOr, CPU.A, TEST_ADDRESS + 1, 2);
}

test "run EOR_ZP" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0x45;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    test_bitop_register(&cpu, .ExclusiveOr, CPU.A, 0x0011, 3);
}

test "run EOR_ZPX" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.X] = 7;
    cpu.memory.data[TEST_ADDRESS + 0] = 0x55;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    test_bitop_register(&cpu, .ExclusiveOr, CPU.A, 0x0011 + 7, 4);
}

test "run EOR_ABS" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0x4D;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_bitop_register(&cpu, .ExclusiveOr, CPU.A, 0x8311, 4);
}

test "run EOR_ABSX same page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.X] = 7;
    cpu.memory.data[TEST_ADDRESS + 0] = 0x5D;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_bitop_register(&cpu, .ExclusiveOr, CPU.A, 0x8311 + 7, 4);
}

test "run EOR_ABSY same page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.Y] = 7;
    cpu.memory.data[TEST_ADDRESS + 0] = 0x59;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_bitop_register(&cpu, .ExclusiveOr, CPU.A, 0x8311 + 7, 4);
}

test "run EOR_ABSX cross page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.X] = 0xFE;
    cpu.memory.data[TEST_ADDRESS + 0] = 0x5D;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_bitop_register(&cpu, .ExclusiveOr, CPU.A, 0x8311 + 0xFE, 5);
}

test "run EOR_ABSY cross page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.Y] = 0xFE;
    cpu.memory.data[TEST_ADDRESS + 0] = 0x59;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_bitop_register(&cpu, .ExclusiveOr, CPU.A, 0x8311 + 0xFE, 5);
}

test "run EOR_INDX" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.X] = 4;
    cpu.memory.data[TEST_ADDRESS + 0] = 0x41;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x20;
    cpu.memory.data[0x20 + 4 + 0] = 0x74;
    cpu.memory.data[0x20 + 4 + 1] = 0x20;
    test_bitop_register(&cpu, .ExclusiveOr, CPU.A, 0x2074, 6);
}

test "run EOR_INDY same page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.Y] = 0x10;
    cpu.memory.data[TEST_ADDRESS + 0] = 0x51;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x86;
    cpu.memory.data[0x86 + 0] = 0x28;
    cpu.memory.data[0x86 + 1] = 0x40;
    test_bitop_register(&cpu, .ExclusiveOr, CPU.A, 0x4028 + 0x10, 5);
}

test "run EOR_INDY cross page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.Y] = 0xFE;
    cpu.memory.data[TEST_ADDRESS + 0] = 0x51;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x86;
    cpu.memory.data[0x86 + 0] = 0x28;
    cpu.memory.data[0x86 + 1] = 0x40;
    test_bitop_register(&cpu, .ExclusiveOr, CPU.A, 0x4028 + 0xFE, 6);
}

// ORA tests

test "run ORA_IMM" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0x09;
    test_bitop_register(&cpu, .InclusiveOr, CPU.A, TEST_ADDRESS + 1, 2);
}

test "run ORA_ZP" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0x05;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    test_bitop_register(&cpu, .InclusiveOr, CPU.A, 0x0011, 3);
}

test "run ORA_ZPX" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.X] = 7;
    cpu.memory.data[TEST_ADDRESS + 0] = 0x15;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    test_bitop_register(&cpu, .InclusiveOr, CPU.A, 0x0011 + 7, 4);
}

test "run ORA_ABS" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0x0D;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_bitop_register(&cpu, .InclusiveOr, CPU.A, 0x8311, 4);
}

test "run ORA_ABSX same page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.X] = 7;
    cpu.memory.data[TEST_ADDRESS + 0] = 0x1D;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_bitop_register(&cpu, .InclusiveOr, CPU.A, 0x8311 + 7, 4);
}

test "run ORA_ABSY same page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.Y] = 7;
    cpu.memory.data[TEST_ADDRESS + 0] = 0x19;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_bitop_register(&cpu, .InclusiveOr, CPU.A, 0x8311 + 7, 4);
}

test "run ORA_ABSX cross page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.X] = 0xFE;
    cpu.memory.data[TEST_ADDRESS + 0] = 0x1D;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_bitop_register(&cpu, .InclusiveOr, CPU.A, 0x8311 + 0xFE, 5);
}

test "run ORA_ABSY cross page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.Y] = 0xFE;
    cpu.memory.data[TEST_ADDRESS + 0] = 0x19;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_bitop_register(&cpu, .InclusiveOr, CPU.A, 0x8311 + 0xFE, 5);
}

test "run ORA_INDX" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.X] = 4;
    cpu.memory.data[TEST_ADDRESS + 0] = 0x01;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x20;
    cpu.memory.data[0x20 + 4 + 0] = 0x74;
    cpu.memory.data[0x20 + 4 + 1] = 0x20;
    test_bitop_register(&cpu, .InclusiveOr, CPU.A, 0x2074, 6);
}

test "run ORA_INDY same page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.Y] = 0x10;
    cpu.memory.data[TEST_ADDRESS + 0] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x86;
    cpu.memory.data[0x86 + 0] = 0x28;
    cpu.memory.data[0x86 + 1] = 0x40;
    test_bitop_register(&cpu, .InclusiveOr, CPU.A, 0x4028 + 0x10, 5);
}

test "run ORA_INDY cross page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.Y] = 0xFE;
    cpu.memory.data[TEST_ADDRESS + 0] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x86;
    cpu.memory.data[0x86 + 0] = 0x28;
    cpu.memory.data[0x86 + 1] = 0x40;
    test_bitop_register(&cpu, .InclusiveOr, CPU.A, 0x4028 + 0xFE, 6);
}

// BIT tests

test "run BIT_ZP" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0x24;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    test_bitop_register(&cpu, .Bit, CPU.A, 0x0011, 3);
}

test "run BIT_ABS" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0x2C;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_bitop_register(&cpu, .Bit, CPU.A, 0x8311, 4);
}

// NOP tests

test "run NOP" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0xEA;
    const used = cpu.run(2);
    testing.expect(used == 2);
}
