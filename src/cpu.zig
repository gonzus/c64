const std = @import("std");
const Type = @import("types.zig").Type;
const Status = @import("status.zig").Status;
const Memory = @import("memory.zig").Memory;
const testing = std.testing;

const allocator = std.heap.page_allocator;

pub const CPU = struct {
    PC: Type.Byte,
    PS: Status,
    SP: Type.Byte,

    memory: Memory,

    A: Type.Byte,
    X: Type.Byte,
    Y: Type.Byte,

    pub fn init() CPU {
        var self = CPU{
            .PC = undefined,
            .PS = undefined,
            .SP = undefined,
            .memory = undefined,
            .A = undefined,
            .X = undefined,
            .Y = undefined,
        };
        self.reset(0);
        return self;
    }

    pub fn reset(self: *CPU, address: Type.Byte) void {
        self.PC = address;
        self.PS.clear();
        self.SP = 0;
        self.A = 0;
        self.X = 0;
        self.Y = 0;
        self.memory.clear();
    }
};

test "create CPU" {
    var cpu = CPU.init();
    testing.expect(cpu.PC == 0);
}
