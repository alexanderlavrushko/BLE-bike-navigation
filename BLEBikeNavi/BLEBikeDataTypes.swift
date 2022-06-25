//
//  BLEBikeDataTypes.swift
//  BLEBikeNavi
//
//  Created by Alexander Lavrushko on 28/03/2021.
//

import Foundation

enum BLEBikeCommand: UInt8 {
    case newFrameWithColor = 1
    case showCurrentFrame = 2
    case drawLine = 3
    case drawCircle = 4
    case fillCircle = 5
    case fillTriangle = 6
    case timeText = 7

    var data: Data {
        Data([self.rawValue])
    }
}

struct BikePoint {
    var x = Int16(0)
    var y = Int16(0)

    init(x: Int16, y: Int16) {
        self.x = x
        self.y = y
    }

    var data: Data {
        var x = self.x
        var data = Data(bytes: &x, count: MemoryLayout<Int16>.size)
        var y = self.y
        data.append(Data(bytes: &y, count: MemoryLayout<Int16>.size))
        return data
    }
}

struct BikeColor {
    var r = UInt8(0)
    var g = UInt8(0)
    var b = UInt8(0)

    init(r: UInt8, g: UInt8, b: UInt8) {
        self.r = r
        self.g = g
        self.b = b
    }

    var color565: UInt16 {
        ((UInt16(r) & 0xF8) << 8) | ((UInt16(g) & 0xFC) << 3) | (UInt16(b) >> 3)
    }

    var data: Data {
        var color565 = self.color565
        return Data(bytes: &color565, count: MemoryLayout<UInt16>.size)
    }

    func multipliedBy(_ intensity: Double) -> BikeColor {
        let newR = (Double(r) * intensity).safeUInt8
        let newG = (Double(g) * intensity).safeUInt8
        let newB = (Double(b) * intensity).safeUInt8
        return BikeColor(r: newR, g: newG, b: newB)
    }
}

struct BikeInfo {
    let screenWidth: Int16
    let screenHeight: Int16
}

private extension Double {
    var safeUInt8: UInt8 {
        if self < Double(UInt8.min) {
            return UInt8.min
        } else if self > Double(UInt8.max) {
            return UInt8.max
        }
        return UInt8(self)
    }
}
