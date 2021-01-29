const std = @import("std");
const Type = @import("types.zig").Type;
const CPU = @import("cpu.zig").CPU;
const testing = std.testing;

test "functional tests" {
    const address: Type.Word = 0x0400;
    var cpu = CPU.init();
    cpu.reset(address);

    var file = try std.fs.cwd().openFile("../data/6502_functional_test.bin", .{ .read = true });
    defer file.close();

    const file_size = try file.getEndPos();
    std.debug.print("File size {d}\n", .{file_size});
    const bytes_read = try file.read(cpu.memory.data[0x000A .. 0x000A + file_size]);
    var lastPC: Type.Word = 0;
    while (true) {
        // run for at least 1 tick
        const used = cpu.run(1);
        // std.debug.print("Ran for {} ticks, PC is now {x}\n", .{ used, cpu.PC });
        if (lastPC == cpu.PC) {
            // std.debug.print("STUCK AT 0x{x}\n", .{cpu.PC});
            break;
        }
        lastPC = cpu.PC;
    }
    testing.expect(lastPC == 0x336d);
}
