//
//  BikeColorScheme.swift
//  BLEBikeNavi
//
//  Created by Alexander Lavrushko on 15/05/2021.
//

import Foundation

struct BikeColorScheme {
    let background: BikeColor
    let route: BikeColor
    let positionIndicator: BikeColor
    let recentPoints: BikeColor
    let accuracyCircle: BikeColor
    let lineToClosestPoint: BikeColor
    let regularText: BikeColor

    init(_ type: ColorScheme) {
        switch type {
        case .dark:
            background = BikeColor(r: 0, g: 0, b: 0)
            route = BikeColor(r: 200, g: 100, b: 0)
            positionIndicator = BikeColor(r: 255, g: 255, b: 255)
            recentPoints = BikeColor(r: 0, g: 255, b: 0)
            accuracyCircle = BikeColor(r: 70, g: 70, b: 70)
            lineToClosestPoint = BikeColor(r: 100, g: 100, b: 100)
            regularText = positionIndicator
        case .light:
            background = BikeColor(r: 255, g: 255, b: 255)
            route = BikeColor(r: 60, g: 0, b: 160)
            positionIndicator = BikeColor(r: 255, g: 98, b: 22)
            recentPoints = BikeColor(r: 0, g: 255, b: 0)
            accuracyCircle = BikeColor(r: 150, g: 150, b: 150)
            lineToClosestPoint = BikeColor(r: 100, g: 100, b: 100)
            regularText = BikeColor(r: 0, g: 0, b: 0)
        }
    }
}
