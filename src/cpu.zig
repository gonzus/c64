const std = @import("std");
const Type = @import("types.zig").Type;
const Status = @import("status.zig").Status;
const Memory = @import("memory.zig").Memory;
const testing = std.testing;

const allocator = std.heap.page_allocator;

pub const CPU = struct {
    const INITIAL_ADDRESS = 0xF000;

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
        self.regs[SP] = 0;
        self.regs[A] = 0;
        self.regs[X] = 0;
        self.regs[Y] = 0;
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
                    self.fetch(.Immediate, A);
                },
                OP.LDA_ZP => {
                    self.fetch(.ZeroPage, A);
                },
                OP.LDA_ZPX => {
                    self.fetch(.ZeroPageX, A);
                },
                OP.LDA_ABS => {
                    self.fetch(.Absolute, A);
                },
                OP.LDA_ABSX => {
                    self.fetch(.AbsoluteX, A);
                },
                OP.LDA_ABSY => {
                    self.fetch(.AbsoluteY, A);
                },
                OP.LDA_XR => {
                    self.fetch(.IndexedIndirect, A);
                },
                OP.LDA_RX => {
                    self.fetch(.IndirectIndexed, A);
                },
                OP.NOP => {
                    self.tick();
                },
            }
        }
        return self.ticks - start;
    }

    fn fetch(self: *CPU, mode: AddressingMode, register: usize) void {
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
                address +%= self.regs[X];
                self.tick();
                break :blk self.read(address);
            },
            .ZeroPageY => blk: {
                var address = @as(Type.Word, self.read(self.PC));
                self.PC += 1;
                address +%= self.regs[Y];
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
                const final = initial + self.regs[X];
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
                const final = initial + self.regs[Y];
                if (!samePage(initial, final)) {
                    self.tick();
                }
                break :blk self.read(final);
            },
            .IndexedIndirect => blk: {
                var address = @as(Type.Word, self.read(self.PC));
                self.PC += 1;
                address +%= self.regs[X];
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
                const final = initial + self.regs[Y];
                if (!samePage(initial, final)) {
                    self.tick();
                }
                break :blk self.read(final);
            },
        };
        self.setNZ(fetched);
        self.regs[register] = fetched;
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
        self.PS.bits.N = if ((value & 0b10000000) > 0) 1 else 0;
        self.PS.bits.Z = if ((value | 0b00000000) > 0) 0 else 1;
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
        cpu.memory.data[address] = d.v;
        const prevP = cpu.PS;
        const prevRegs = cpu.regs;
        cpu.PS.byte = 0;
        const used = cpu.run(ticks);
        testing.expect(used == ticks);
        testing.expect(cpu.regs[register] == d.v);
        testing.expect(cpu.PS.bits.N == d.N);
        testing.expect(cpu.PS.bits.Z == d.Z);
        testing.expect(cpu.PS.bits.C == prevP.bits.C);
        testing.expect(cpu.PS.bits.I == prevP.bits.I);
        testing.expect(cpu.PS.bits.D == prevP.bits.D);
        testing.expect(cpu.PS.bits.B == prevP.bits.B);
        testing.expect(cpu.PS.bits.V == prevP.bits.V);
        testing.expect(register == CPU.A or cpu.regs[CPU.A] == prevRegs[CPU.A]);
        testing.expect(register == CPU.X or cpu.regs[CPU.X] == prevRegs[CPU.X]);
        testing.expect(register == CPU.Y or cpu.regs[CPU.Y] == prevRegs[CPU.Y]);
        testing.expect(register == CPU.SP or cpu.regs[CPU.SP] == prevRegs[CPU.SP]);
    }
}

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

test "run NOP" {
    var cpu = CPU.init();
    cpu.reset(TEST_ADDRESS);
    cpu.memory.data[TEST_ADDRESS + 0] = 0xEA;
    const used = cpu.run(2);
    testing.expect(used == 2);
}
