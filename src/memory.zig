const std = @import("std");
const Type = @import("types.zig").Type;
const testing = std.testing;

const allocator = std.heap.page_allocator;

pub const Memory = struct {
    const SIZE = 0xFFFF;

    data: [SIZE]Type.Byte,

    pub fn init() Memory {
        var self = Memory{
            .data = undefined,
        };
        self.clear();
        return self;
    }

    pub fn clear(self: *Memory) void {
        for (self.data) |*d| {
            d.* = 0;
        }
    }
};

test "create memory" {
    var memory = Memory.init();
    testing.expect(memory.data[0] == 0);
    testing.expect(memory.data[Memory.SIZE - 1] == 0);
    memory.data[0x1234] = 0x11;
    testing.expect(memory.data[0x1234] == 0x11);
}
