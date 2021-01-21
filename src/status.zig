const std = @import("std");
const Type = @import("types.zig").Type;
const testing = std.testing;

const allocator = std.heap.page_allocator;

pub const Status = extern union {
    byte: Type.Byte,
    bits: Bits,

    const Name = enum {
        Carry = 0,
        Zero = 1,
        Interrupt = 2,
        Decimal = 3,
        Break = 4,
        UNUSED = 5,
        Overflow = 6,
        Negative = 7,
    };

    const Bits = packed struct {
        C: Type.Bit,
        Z: Type.Bit,
        I: Type.Bit,
        D: Type.Bit,
        B: Type.Bit,
        u: Type.Bit,
        V: Type.Bit,
        N: Type.Bit,
    };

    pub fn init() Status {
        return Status.initFromByte(0);
    }

    pub fn initFromByte(byte: Type.Byte) Status {
        var self = Status{
            .byte = byte,
        };
        return self;
    }

    pub fn show(self: Status) void {
        const out = std.io.getStdOut().outStream();
        out.print("C={d} Z={d} I={d} D={d} B={d} V={d} N={d}\n", .{ self.bits.C, self.bits.Z, self.bits.I, self.bits.D, self.bits.B, self.bits.V, self.bits.N }) catch unreachable;
    }

    pub fn getBitByName(self: Status, name: Name) Type.Bit {
        const shift = @enumToInt(name);
        const mask = @as(Type.Byte, 1) << shift;
        return if ((self.byte & mask) > 0) 1 else 0;
    }

    pub fn setBitByName(self: *Status, name: Name, bit: u1) void {
        const shift = @enumToInt(name);
        const mask = @as(Type.Byte, 1) << shift;
        if (bit == 1) {
            self.byte |= mask;
        } else {
            self.byte &= ~mask;
        }
    }

    pub fn clear(self: *Status) void {
        self.byte = 0;
    }
};

test "create zero PS" {
    var PS = Status.init();
    testing.expect(PS.byte == 0);
    testing.expect(PS.bits.C == 0);

    PS.setBitByName(Status.Name.Carry, 1);
    PS.bits.N = 1;
    testing.expect(PS.getBitByName(Status.Name.Carry) == 1);
    testing.expect(PS.bits.C == 1);
    // PS.show();

    PS.bits.C = 0;
    testing.expect(PS.getBitByName(Status.Name.Carry) == 0);
    testing.expect(PS.bits.C == 0);
}

test "create PS with initial byte value" {
    const byte: Type.Byte = 0b10000000;
    var PS = Status.initFromByte(byte);
    testing.expect(PS.byte == byte);
    testing.expect(PS.bits.C == 0);
    testing.expect(PS.bits.N == 1);
    testing.expect(PS.getBitByName(Status.Name.Negative) == 1);
    testing.expect(PS.getBitByName(Status.Name.Carry) == 0);
}

test "create PS with initial bit values" {
    var PS = Status.init();
    PS.bits.C = 1;
    PS.bits.V = 1;
    PS.bits.N = 1;
    // PS.show();
    testing.expect(PS.byte == 0b11000001);
}
