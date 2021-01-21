const std = @import("std");
const Type = @import("types.zig").Type;
const Status = @import("status.zig").Status;
const Memory = @import("memory.zig").Memory;
const testing = std.testing;

const allocator = std.heap.page_allocator;

pub const CPU = struct {
    const INITIAL_ADDRESS = 0xF000;

    PC: Type.Word, // Program Counter
    SP: Type.Byte, // Stack Pointer
    P: Status, // Processor Status

    A: Type.Byte, // Accumulator
    X: Type.Byte, // X register
    Y: Type.Byte, // Y register

    memory: Memory, // Memory bank with 64 KB -- wow

    ticks: u32, // Cycle counter

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

        NOP = 0xEA,
    };

    pub fn init() CPU {
        var self = CPU{
            .PC = undefined,
            .SP = undefined,
            .P = undefined,
            .A = undefined,
            .X = undefined,
            .Y = undefined,
            .memory = undefined,
            .ticks = undefined,
        };
        self.reset(INITIAL_ADDRESS);
        return self;
    }

    pub fn reset(self: *CPU, address: Type.Word) void {
        self.PC = address;
        self.SP = 0;
        self.P.clear();
        self.A = 0;
        self.X = 0;
        self.Y = 0;
        self.memory.clear();
        self.ticks = 0;
    }

    pub fn run(self: *CPU, limit: u32) u32 {
        const start = self.ticks;
        while ((self.ticks - start) < limit) {
            const op = @intToEnum(OP, self.read(self.PC));
            self.PC += 1;
            switch (op) {
                OP.LDA_IMM => {
                    const value = self.fetch(.Immediate);
                    self.A = value;
                    self.setNZ(value);
                },
                OP.LDA_ZP => {
                    const value = self.fetch(.ZeroPage);
                    self.A = value;
                    self.setNZ(value);
                },
                OP.LDA_ZPX => {
                    const value = self.fetch(.ZeroPageX);
                    self.A = value;
                    self.setNZ(value);
                },
                OP.LDA_ABS => {
                    const value = self.fetch(.Absolute);
                    self.A = value;
                    self.setNZ(value);
                },
                OP.LDA_ABSX => {
                    const value = self.fetch(.AbsoluteX);
                    self.A = value;
                    self.setNZ(value);
                },
                OP.LDA_ABSY => {
                    const value = self.fetch(.AbsoluteY);
                    self.A = value;
                    self.setNZ(value);
                },
                OP.LDA_XR => {
                    const value = self.fetch(.IndexedIndirect);
                    self.A = value;
                    self.setNZ(value);
                },
                OP.LDA_RX => {
                    const value = self.fetch(.IndirectIndexed);
                    self.A = value;
                    self.setNZ(value);
                },
                OP.NOP => {
                    self.tick();
                },
            }
        }
        return self.ticks - start;
    }

    fn fetch(self: *CPU, mode: AddressingMode) Type.Byte {
        const fetched = switch (mode) {
            .Immediate => blk: {
                const value = self.read(self.PC);
                self.PC += 1;
                break :blk value;
            },
            .ZeroPage => blk: {
                const address = @as(Type.Word, self.read(self.PC));
                self.PC += 1;
                break :blk self.read(address);
            },
            .ZeroPageX => blk: {
                var address = @as(Type.Word, self.read(self.PC));
                self.PC += 1;
                address +%= self.X;
                self.tick();
                break :blk self.read(address);
            },
            .ZeroPageY => blk: {
                var address = @as(Type.Word, self.read(self.PC));
                self.PC += 1;
                address +%= self.Y;
                self.tick();
                break :blk self.read(address);
            },
            .Absolute => blk: {
                const lo = @as(Type.Word, self.read(self.PC)) << 0;
                self.PC += 1;
                const hi = @as(Type.Word, self.read(self.PC)) << 8;
                self.PC += 1;
                const address = hi | lo;
                break :blk self.read(address);
            },
            .AbsoluteX => blk: {
                const lo = @as(Type.Word, self.read(self.PC)) << 0;
                self.PC += 1;
                const hi = @as(Type.Word, self.read(self.PC)) << 8;
                self.PC += 1;
                const initial = hi | lo;
                const final = initial + self.X;
                if (!samePage(initial, final)) {
                    self.tick();
                }
                break :blk self.read(final);
            },
            .AbsoluteY => blk: {
                const lo = @as(Type.Word, self.read(self.PC)) << 0;
                self.PC += 1;
                const hi = @as(Type.Word, self.read(self.PC)) << 8;
                self.PC += 1;
                const initial = hi | lo;
                const final = initial + self.Y;
                if (!samePage(initial, final)) {
                    self.tick();
                }
                break :blk self.read(final);
            },
            .IndexedIndirect => blk: {
                var address = @as(Type.Word, self.read(self.PC));
                self.PC += 1;
                address +%= self.X;
                self.tick();
                const lo = @as(Type.Word, self.read(address + 0)) << 0;
                const hi = @as(Type.Word, self.read(address + 1)) << 8;
                const final = hi | lo;
                break :blk self.read(final);
            },
            .IndirectIndexed => blk: {
                const address = @as(Type.Word, self.read(self.PC));
                self.PC += 1;
                const lo = @as(Type.Word, self.read(address + 0)) << 0;
                const hi = @as(Type.Word, self.read(address + 1)) << 8;
                const initial = hi | lo;
                const final = initial + self.Y;
                if (!samePage(initial, final)) {
                    self.tick();
                }
                break :blk self.read(final);
            },
        };
        return fetched;
    }

    fn read(self: *CPU, address: Type.Word) Type.Byte {
        const value = self.memory.data[address];
        self.tick();
        return value;
    }

    fn tick(self: *CPU) void {
        self.ticks += 1;
    }

    fn setNZ(self: *CPU, value: Type.Word) void {
        self.P.bits.N = if ((value & 0b10000000) > 0) 1 else 0;
        self.P.bits.Z = if ((value | 0b00000000) > 0) 0 else 1;
    }

    fn samePage(p1: Type.Word, p2: Type.Word) bool {
        // It seems adding something to an address will incur in an extra tick
        // ONLY when that caused the address to cross onto another page.
        return (p1 & 0xFF00) == (p2 & 0xFF00);
    }
};

const TEST_ADDRESS = 0x4433;

test "create CPU" {
    var cpu = CPU.init();
    testing.expect(cpu.PC == CPU.INITIAL_ADDRESS);
    cpu.reset(TEST_ADDRESS);
    testing.expect(cpu.PC == TEST_ADDRESS);
}

fn test_lda(cpu: *CPU, address: Type.Word, ticks: u32) void {
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
        cpu.memory.data[address] = d.v;
        const prevP = cpu.P;
        const prevSP = cpu.SP;
        const prevX = cpu.X;
        const prevY = cpu.Y;
        cpu.P.byte = 0;
        const used = cpu.run(ticks);
        testing.expect(used == ticks);
        testing.expect(cpu.A == d.v);
        testing.expect(cpu.P.bits.N == d.N);
        testing.expect(cpu.P.bits.Z == d.Z);
        testing.expect(cpu.P.bits.C == prevP.bits.C);
        testing.expect(cpu.P.bits.I == prevP.bits.I);
        testing.expect(cpu.P.bits.D == prevP.bits.D);
        testing.expect(cpu.P.bits.B == prevP.bits.B);
        testing.expect(cpu.P.bits.V == prevP.bits.V);
        testing.expect(cpu.SP == prevSP);
        testing.expect(cpu.X == prevX);
        testing.expect(cpu.Y == prevY);
    }
}

test "run LDA_IMM" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0xA9;
    test_lda(&cpu, TEST_ADDRESS + 1, 2);
}

test "run LDA_ZP" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0xA5;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    test_lda(&cpu, 0x0011, 3);
}

test "run LDA_ZPX" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.X = 7;
    cpu.memory.data[TEST_ADDRESS + 0] = 0xB5;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    test_lda(&cpu, 0x0011 + 7, 4);
}

test "run LDA_ABS" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0xAD;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_lda(&cpu, 0x8311, 4);
}

test "run LDA_ABSX same page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.X = 7;
    cpu.memory.data[TEST_ADDRESS + 0] = 0xBD;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_lda(&cpu, 0x8311 + 7, 4);
}

test "run LDA_ABSY same page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.Y = 7;
    cpu.memory.data[TEST_ADDRESS + 0] = 0xB9;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_lda(&cpu, 0x8311 + 7, 4);
}

test "run LDA_ABSX cross page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.X = 0xFE;
    cpu.memory.data[TEST_ADDRESS + 0] = 0xBD;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_lda(&cpu, 0x8311 + 0xFE, 5);
}

test "run LDA_ABSY cross page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.Y = 0xFE;
    cpu.memory.data[TEST_ADDRESS + 0] = 0xB9;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 2] = 0x83;
    test_lda(&cpu, 0x8311 + 0xFE, 5);
}

test "run LDA_XR" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.X = 4;
    cpu.memory.data[TEST_ADDRESS + 0] = 0xA1;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x20;
    cpu.memory.data[0x20 + 4 + 0] = 0x74;
    cpu.memory.data[0x20 + 4 + 1] = 0x20;
    test_lda(&cpu, 0x2074, 6);
}

test "run LDA_RX same page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.Y = 0x10;
    cpu.memory.data[TEST_ADDRESS + 0] = 0xB1;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x86;
    cpu.memory.data[0x86 + 0] = 0x28;
    cpu.memory.data[0x86 + 1] = 0x40;
    test_lda(&cpu, 0x4028 + 0x10, 5);
}

test "run LDA_RX cross page" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.Y = 0xFE;
    cpu.memory.data[TEST_ADDRESS + 0] = 0xB1;
    cpu.memory.data[TEST_ADDRESS + 1] = 0x86;
    cpu.memory.data[0x86 + 0] = 0x28;
    cpu.memory.data[0x86 + 1] = 0x40;
    test_lda(&cpu, 0x4028 + 0xFE, 6);
}

test "run NOP" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0xEA;
    const used = cpu.run(2);
    testing.expect(used == 2);
}
