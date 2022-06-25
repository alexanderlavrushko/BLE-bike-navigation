//
//  Settings.swift
//  BLEBikeNavi
//
//  Created by Alexander Lavrushko on 02/05/2021.
//

import Foundation

enum PositionSource: String {
    case real
    case mapViewCenter
}

enum UpSource: String {
    case northUp
    case systemCourseUp
    case customCourseUp
    case headingUp
}

enum ColorScheme: String {
    case dark
    case light
}

enum SimulatedAccuracy {
    case good
    case bad
}

enum AdditionalData: String {
    case none
    case time
}

enum MapScale: String {
    case tile256
    case tile512
}

protocol SettingsDelegate: AnyObject {
    func settingsDidChange()
    func positionSourceDidChange()
    func mapLookDidChange()
}

enum StorageKey: String {
    case upSource
    case metersPerPixel
    case lineWidthScale
    case colorScheme
    case additionalData
    case mapScale

    var key: String {
        "settings.\(rawValue)"
    }

    var exists: Bool {
        UserDefaults.standard.object(forKey: key) != nil
    }

    func readString() -> String? {
        exists ? UserDefaults.standard.string(forKey: key) : nil
    }

    func readDouble() -> Double? {
        exists ? UserDefaults.standard.double(forKey: key) : nil
    }

    func writeString(_ value: String) {
        UserDefaults.standard.set(value, forKey: key)
        UserDefaults.standard.synchronize()
    }

    func writeDouble(_ value: Double) {
        UserDefaults.standard.set(value, forKey: key)
        UserDefaults.standard.synchronize()
    }
}

class Settings {
    private(set) static var instance: Settings?

    static func create() {
        guard instance == nil else { return }
        instance = Settings()
    }

    static func destroy() {
        Settings.instance = nil
    }

    weak var delegate: SettingsDelegate?
    
    // default values
    static var defaultPositionSource: PositionSource { .real }
    static var defaultUpSource: UpSource { .customCourseUp }
    static var defaultMetersPerPixel: Double { 1.5 }
    static var defaultLineWidthScale: Double { 2 }
    static var defaultColorScheme: ColorScheme { .dark }
    static var defaultSimulatedAccuracy: SimulatedAccuracy { .good }
    static var defaultAdditionalData: AdditionalData { .time }
    static var defaultMapScale: MapScale { .tile512 }

    // settings properties
    var positionSource = defaultPositionSource {
        didSet {
            delegate?.positionSourceDidChange()
        }
    }
    var upSource = defaultUpSource {
        didSet {
            StorageKey.upSource.writeString(upSource.rawValue)
            delegate?.settingsDidChange()
        }
    }
    var metersPerPixel = defaultMetersPerPixel {
        didSet {
            StorageKey.metersPerPixel.writeDouble(metersPerPixel)
            delegate?.settingsDidChange()
        }
    }
    var lineWidthScale = defaultLineWidthScale {
        didSet {
            StorageKey.lineWidthScale.writeDouble(lineWidthScale)
            delegate?.settingsDidChange()
        }
    }
    var colorScheme = defaultColorScheme {
        didSet {
            StorageKey.colorScheme.writeString(colorScheme.rawValue)
            delegate?.settingsDidChange()
        }
    }
    var simulatedAccuracy = defaultSimulatedAccuracy
    var additionalData = defaultAdditionalData {
        didSet {
            StorageKey.additionalData.writeString(additionalData.rawValue)
            delegate?.settingsDidChange()
        }
    }
    var mapScale = defaultMapScale {
        didSet {
            StorageKey.mapScale.writeString(mapScale.rawValue)
            delegate?.mapLookDidChange()
        }
    }

    // methods
    private init() {
        if let storedUpSource = UpSource(rawValue: StorageKey.upSource.readString() ?? "") {
            upSource = storedUpSource
        }
        if let storedMetersPerPixel = StorageKey.metersPerPixel.readDouble() {
            metersPerPixel = storedMetersPerPixel
        }
        if let storedLineWidthScale = StorageKey.lineWidthScale.readDouble() {
            lineWidthScale = storedLineWidthScale
        }
        if let storedColorScheme = ColorScheme(rawValue: StorageKey.colorScheme.readString() ?? "") {
            colorScheme = storedColorScheme
        }
        if let storedAdditionalData = AdditionalData(rawValue: StorageKey.additionalData.readString() ?? "") {
            additionalData = storedAdditionalData
        }
        if let storedMapScale = MapScale(rawValue: StorageKey.mapScale.readString() ?? "") {
            mapScale = storedMapScale
        }
    }

    func switchPositionSource() {
        if positionSource == .real {
            positionSource = .mapViewCenter
        } else {
            positionSource = .real
        }
    }

    func switchUpSource() {
        if upSource == .northUp {
            upSource = .systemCourseUp
        } else if upSource == .systemCourseUp {
            upSource = .customCourseUp
        } else if upSource == .customCourseUp {
            upSource = .headingUp
        } else {
            upSource = .northUp
        }
    }

    func switchMetersPerPixel() {
        if metersPerPixel == 0.67 {
            metersPerPixel = 1
        } else if metersPerPixel == 1 {
            metersPerPixel = 1.5
        } else if metersPerPixel == 1.5 {
            metersPerPixel = 2
        } else if metersPerPixel == 2 {
            metersPerPixel = 3
        } else if metersPerPixel == 3 {
            metersPerPixel = 4
        } else {
            metersPerPixel = 0.67
        }
    }

    func switchLineWidthScale() {
        if lineWidthScale == 1 {
            lineWidthScale = 2
        } else if lineWidthScale == 2 {
            lineWidthScale = 3
        } else if lineWidthScale == 3 {
            lineWidthScale = 4
        } else if lineWidthScale == 4 {
            lineWidthScale = 5
        } else if lineWidthScale == 5 {
            lineWidthScale = 6
        } else {
            lineWidthScale = 1
        }
    }

    func switchColorScheme() {
        if colorScheme == .dark {
            colorScheme = .light
        } else {
            colorScheme = .dark
        }
    }

    func switchAdditionalData() {
        if additionalData == .none {
            additionalData = .time
        } else {
            additionalData = .none
        }
    }

    func switchMapScale() {
        if mapScale == .tile256 {
            mapScale = .tile512
        } else {
            mapScale = .tile256
        }
    }
}
