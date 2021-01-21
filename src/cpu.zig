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
        Absolute,
    };

    const OP = enum(Type.Byte) {
        LDA_IMM = 0xA9,
        LDA_ZP = 0xA5,
        LDA_ABS = 0xAD,

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
            const op = @intToEnum(OP, self.fetch(.Immediate));
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
                OP.LDA_ABS => {
                    const value = self.fetch(.Absolute);
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
            .Immediate => self.memory.data[self.PC],
            .ZeroPage => blk: {
                const address = @as(Type.Word, self.memory.data[self.PC + 0]);
                self.tick();
                break :blk self.memory.data[address];
            },
            .Absolute => blk: {
                const lo = @as(Type.Word, self.memory.data[self.PC + 0]) << 0;
                const hi = @as(Type.Word, self.memory.data[self.PC + 1]) << 8;
                const address = hi | lo;
                self.tick();
                self.tick();
                break :blk self.memory.data[address];
            },
        };
        self.tick();
        self.PC += 1;
        return fetched;
    }

    fn tick(self: *CPU) void {
        self.ticks += 1;
    }

    fn setNZ(self: *CPU, value: Type.Word) void {
        self.P.bits.N = if ((value & 0b10000000) > 0) 1 else 0;
        self.P.bits.Z = if ((value | 0b00000000) > 0) 0 else 1;
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
        const used = cpu.run(ticks);
        testing.expect(used == ticks);
        testing.expect(cpu.A == d.v);
        testing.expect(cpu.P.bits.N == d.N);
        testing.expect(cpu.P.bits.Z == d.Z);
    }
}

test "run LDA_IMM" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0x00] = 0xA9;
    test_lda(&cpu, TEST_ADDRESS + 0x01, 2);
}

test "run LDA_ZP" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0x00] = 0xA5;
    cpu.memory.data[TEST_ADDRESS + 0x01] = 0x11;
    test_lda(&cpu, 0x0011, 3);
}

test "run LDA_ABS" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0x00] = 0xAD;
    cpu.memory.data[TEST_ADDRESS + 0x01] = 0x11;
    cpu.memory.data[TEST_ADDRESS + 0x02] = 0x83;
    test_lda(&cpu, 0x8311, 4);
}

test "run NOP" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0x00] = 0xEA;
    const used = cpu.run(2);
    testing.expect(used == 2);
}
