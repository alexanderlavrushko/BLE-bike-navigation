//
//  BLEBikeDataTypes.swift
//  BLEBikeNavi
//
//  Created by Alexander Lavrushko on 28/03/2021.
//

import Foundation

class BLEBikeDataEncoder {
    func buildNewFrame(color: BikeColor) -> Data {
        var data = BLEBikeCommand.newFrameWithColor.data
        data.append(color.data)
        return data
    }

    func buildDrawLine(from: BikePoint, to: BikePoint, color: BikeColor, width: UInt8) -> Data {
        var data = BLEBikeCommand.drawLine.data
        data.append(from.data)
        data.append(to.data)
        data.append(color.data)
        data.append(contentsOf: [width])
        return data
    }

    func buildShowFrame() -> Data {
        BLEBikeCommand.showCurrentFrame.data
    }

    func buildDrawCircle(center: BikePoint, radius: UInt8, color: BikeColor) -> Data {
        var data = BLEBikeCommand.drawCircle.data
        data.append(center.data)
        data.append(contentsOf: [radius])
        data.append(color.data)
        return data
    }

    func buildFillCircle(center: BikePoint, radius: UInt8, color: BikeColor) -> Data {
        var data = BLEBikeCommand.fillCircle.data
        data.append(center.data)
        data.append(contentsOf: [radius])
        data.append(color.data)
        return data
    }

    func buildFillTriangle(_ p1: BikePoint, _ p2: BikePoint, _ p3: BikePoint, color: BikeColor) -> Data {
        var data = BLEBikeCommand.fillTriangle.data
        data.append(p1.data)
        data.append(p2.data)
        data.append(p3.data)
        data.append(color.data)
        return data
    }

    func buildTimeText(_ timeText: String, color: BikeColor) -> Data {
        var data = BLEBikeCommand.timeText.data
        data.append(color.data)
        if let timeData = timeText.data(using: String.Encoding.utf8) {
            data.append(timeData)
        }
        data.append(UInt8(0))
        return data
    }

    func decodeBikeInfo(_ data: Data?) -> BikeInfo? {
        guard let data = data else {
            return nil
        }
        guard data.count >= 2 else {
            return nil
        }
        return BikeInfo(screenWidth: Int16(data[0]), screenHeight: Int16(data[1]))
    }
}
