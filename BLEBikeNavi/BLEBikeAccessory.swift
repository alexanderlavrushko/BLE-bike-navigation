//
//  BLEBikeDataTypes.swift
//  BLEBikeNavi
//
//  Created by Alexander Lavrushko on 28/03/2021.
//

import Foundation
import UIKit
import CoreBluetooth

protocol BLEBikeDelegate: AnyObject {
    func stateDidChange(_ newState: BLEBikeAccessoryState)
}

class BLEBikeAccessory: NSObject {
    private(set) static var instance: BLEBikeAccessory?
    weak var delegate: BLEBikeDelegate?

    // private properties
    private let uuidService = CBUUID(string: "1E6387F0-BE8C-40DA-8F76-8ED84C42065D")
    private let uuidCharReadInfo = CBUUID(string: "1E6387F1-BE8C-40DA-8F76-8ED84C42065D")
    private let uuidCharWriteData = CBUUID(string: "1E6387F2-BE8C-40DA-8F76-8ED84C42065D")
    private let uuidCharIndicateRequest = CBUUID(string: "1E6387F3-BE8C-40DA-8F76-8ED84C42065D")

    private let serialQueue: DispatchQueue
    private var centralManager: CBCentralManager!
    private var connectedPeripheral: CBPeripheral?

    private var encoder = BLEBikeDataEncoder()
    private var latestData = Data()

    private var state = BLEBikeAccessoryState.bluetoothNotReady {
        didSet {
            guard state != oldValue else { return }
            log("accessory state = \(state.humanReadableString)")
            if delegate != nil {
                let newState = state
                DispatchQueue.main.async {
                    self.delegate?.stateDidChange(newState)
                }
            }
        }
    }

    // methods
    override private init() {
        serialQueue = DispatchQueue(label: "BLEBike.serialQueue")
        super.init()
        centralManager = CBCentralManager(delegate: self, queue: serialQueue)
        log("BLEBikeAccessory init done")
    }

    deinit {
        log("BLEBikeAccessory deinit")
    }

    public static func createAndStart() {
        guard instance == nil else { return }
        instance = BLEBikeAccessory()
    }

    public static func stopAndDestroy() {
        BLEBikeAccessory.instance = nil
    }

    public func getState(withCompletion completion: @escaping (BLEBikeAccessoryState) -> Void) {
        serialQueue.async {
            let currentState = self.state
            DispatchQueue.main.async {
                completion(currentState)
            }
        }
    }

    public func fillScreen(color: BikeColor) {
        let data = encoder.buildFillScreen(color: color)
        writeProperty(uuid: uuidCharWriteData, data: data)
    }

    public func drawLine(from: BikePoint, to: BikePoint, color: BikeColor, width: UInt8) {
        let data = encoder.buildDrawLine(from: from, to: to, color: color, width: width)
        writeProperty(uuid: uuidCharWriteData, data: data)
    }
}

// MARK: - Private methods
private extension BLEBikeAccessory {
    func log(_ message: String) {
        print("BLE_LOG: \(message)")
    }
}

// MARK: - BLE related commands
private extension BLEBikeAccessory {
    func startScanning() {
        state = .scanning
        centralManager.scanForPeripherals(withServices: [uuidService], options: nil)
    }

    func handleConnectionSetupFinished() {
        state = .connected
    }

    func readProperty(uuid: CBUUID) {
        guard let characteristic = getCharacteristic(uuid: uuid) else {
            log("ERROR: read failed, characteristic unavailable, uuid = \(uuid.uuidString)")
            return
        }
        connectedPeripheral?.readValue(for: characteristic)
    }

    func writeProperty(uuid: CBUUID, data: Data) {
        guard let characteristic = getCharacteristic(uuid: uuid) else {
            log("ERROR: write failed, characteristic unavailable, uuid = \(uuid.uuidString)")
            return
        }
        connectedPeripheral?.writeValue(data, for: characteristic, type: .withResponse)
    }

    func getCharacteristic(uuid: CBUUID) -> CBCharacteristic? {
        guard let service = connectedPeripheral?.services?.first(where: { $0.uuid == uuidService }) else {
            return nil
        }
        return service.characteristics?.first(where: { $0.uuid == uuid })
    }

    func setSubscription(_ newState: Bool, uuid: CBUUID) {
        guard let peripheral = connectedPeripheral else {
            log("ERROR: subscribe failed, connectedPeripheral is nil")
            return
        }
        guard let characteristic = getCharacteristic(uuid: uuid) else {
            log("ERROR: subscribe failed, characteristic unavailable, uuid = \(uuid.uuidString)")
            return
        }
        guard characteristic.properties.contains(.indicate) else {
            log("ERROR: subscribe failed, characteristic doesn't have indicate property, uuid = \(uuid.uuidString)")
            return
        }
        peripheral.setNotifyValue(newState, for: characteristic)
    }
}

// MARK: - BLE Central delegate
extension BLEBikeAccessory: CBCentralManagerDelegate {

    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        log("central.state = \(central.state.humanReadableString)")

        if central.state == .poweredOn {
            startScanning()
        } else {
            state = .bluetoothNotReady
        }
    }

    func centralManager(_ central: CBCentralManager,
                        didDiscover peripheral: CBPeripheral,
                        advertisementData: [String: Any], rssi RSSI: NSNumber) {
        log("didDiscover {name = \(peripheral.name ?? String("nil")), state = \(peripheral.state.humanReadableString)}")

        guard connectedPeripheral == nil else {
            log("didDiscover ignored (connectedPeripheral already set)")
            return
        }

        connectedPeripheral = peripheral
        state = .connecting
        centralManager.stopScan()
        centralManager.connect(peripheral, options: nil)
    }

    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        log("didConnect")
        state = .connectedPreparing

        connectedPeripheral = peripheral
        peripheral.delegate = self
        peripheral.discoverServices([uuidService])
    }

    func centralManager(_ central: CBCentralManager, didFailToConnect peripheral: CBPeripheral, error: Error?) {
        log("didFailToConnect, error = \(String(describing: error))")
        connectedPeripheral = nil
        startScanning()
    }

    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        log("didDisconnectPeripheral, error = \(String(describing: error))")
        connectedPeripheral = nil
        startScanning()
    }
}

// MARK: - BLE Peripheral delegate
extension BLEBikeAccessory: CBPeripheralDelegate {
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        guard let service = peripheral.services?.first(where: { $0.uuid == uuidService }) else {
            log("ERROR: didDiscoverServices, service NOT found")
            return
        }

        log("didDiscoverServices, service found")
        peripheral.discoverCharacteristics([uuidCharReadInfo, uuidCharWriteData, uuidCharIndicateRequest], for: service)
    }

    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        log("didDiscoverCharacteristics \(error == nil ? "OK" : "error: \(String(describing: error))")")
        setSubscription(true, uuid: uuidCharIndicateRequest)
    }

    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
        guard error == nil else {
            log("ERROR: didUpdateValueFor error = \(String(describing: error))")
            return
        }

//        if characteristic.uuid == uuidCharIndicateRequest {
//            serialQueue.async {
//            }
//        }
    }

    func peripheral(_ peripheral: CBPeripheral, didWriteValueFor characteristic: CBCharacteristic, error: Error?) {
        if let error = error {
            log("ERROR: didWriteValueFor: \(String(describing: error))")
        }
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didUpdateNotificationStateFor characteristic: CBCharacteristic,
                    error: Error?) {
        guard error == nil else {
            log("""
                ERROR: didUpdateNotificationStateFor uuid = \(characteristic.uuid.uuidString), \
                error = \(String(describing: error))
                """)
            return
        }

        if characteristic.uuid == uuidCharIndicateRequest, characteristic.isNotifying {
            serialQueue.async {
                self.handleConnectionSetupFinished()
            }
        } else {
            log("""
                WARN: unexpected didUpdateNotificationStateFor uuid = \(characteristic.uuid.uuidString), \
                isNotifying = \(characteristic.isNotifying)
                """)
        }
    }
}
