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

    const AddressingMode = enum {
        Immediate,
        ZeroPage,
        ZeroPageX,
        ZeroPageY,
        Absolute,
        AbsoluteX,
        AbsoluteY,
        IndexedIndirect,
        IndirectIndexed,
    };

    const OP = enum(Type.Byte) {
        LDA_IMM = 0xA9,
        LDA_ZP = 0xA5,
        LDA_ZPX = 0xB5,
        LDA_ABS = 0xAD,
        LDA_ABSX = 0xBD,
        LDA_ABSY = 0xB9,
        LDA_XR = 0xA1,
        LDA_RX = 0xB1,

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
        STA_XR = 0x81,
        STA_RX = 0x91,

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
                OP.LDA_XR => self.fetch(A, .IndexedIndirect),
                OP.LDA_RX => self.fetch(A, .IndirectIndexed),

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
                OP.STA_XR => self.store(A, .IndexedIndirect),
                OP.STA_RX => self.store(A, .IndirectIndexed),

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

                OP.NOP => self.tick(),
            }
        }
        return self.ticks - start;
    }

    fn fetch(self: *CPU, register: usize, mode: AddressingMode) void {
        const address = self.computeAddress(mode);
        const value = self.readByte(address);
        self.regs[register] = value;
        self.setNZ(self.regs[register]);
    }

    fn store(self: *CPU, register: usize, mode: AddressingMode) void {
        const address = self.computeAddress(mode);
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

    fn computeAddress(self: *CPU, mode: AddressingMode) Type.Word {
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
                if (!samePage(initial, final)) {
                    self.tick();
                }
                break :blk final;
            },
            .AbsoluteY => blk: {
                const initial = self.readWord(self.PC);
                const final = initial + self.regs[Y];
                if (!samePage(initial, final)) {
                    self.tick();
                }
                break :blk final;
            },
            .IndexedIndirect => blk: {
                var address = @as(Type.Word, self.readByte(self.PC));
                self.PC += 1;
                address +%= self.regs[X];
                self.tick();
                const final = self.readWord(address);
                break :blk final;
            },
            .IndirectIndexed => blk: {
                const address = @as(Type.Word, self.readByte(self.PC));
                self.PC += 1;
                const initial = self.readWord(address);
                const final = initial + self.regs[Y];
                if (!samePage(initial, final)) {
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

test "run LDA_XR" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.X] = 4;
    cpu.memory.data[TEST_ADDRESS + 0] = 0xA1;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x20;
    cpu.memory.data[0x20 + 4 + 0] = 0x74;
    cpu.memory.data[0x20 + 4 + 1] = 0x20;
    test_load_register(&cpu, CPU.A, 0x2074, 6);
}

test "run LDA_RX same page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.Y] = 0x10;
    cpu.memory.data[TEST_ADDRESS + 0] = 0xB1;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x86;
    cpu.memory.data[0x86 + 0] = 0x28;
    cpu.memory.data[0x86 + 1] = 0x40;
    test_load_register(&cpu, CPU.A, 0x4028 + 0x10, 5);
}

test "run LDA_RX cross page" {
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

test "run STA_XR" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.X] = 4;
    cpu.memory.data[TEST_ADDRESS + 0] = 0x81;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x20;
    cpu.memory.data[0x20 + 4 + 0] = 0x74;
    cpu.memory.data[0x20 + 4 + 1] = 0x20;
    test_save_register(&cpu, CPU.A, 0x2074, 6);
}

test "run STA_RX same page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.regs[CPU.Y] = 0x10;
    cpu.memory.data[TEST_ADDRESS + 0] = 0x91;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x86;
    cpu.memory.data[0x86 + 0] = 0x28;
    cpu.memory.data[0x86 + 1] = 0x40;
    test_save_register(&cpu, CPU.A, 0x4028 + 0x10, 5);
}

test "run STA_RX cross page" {
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

// NOP tests

test "run NOP" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0xEA;
    const used = cpu.run(2);
    testing.expect(used == 2);
}
