//
//  MenuViewController.swift
//  BLEBikeNavi
//
//  Created by Alexander Lavrushko on 02/05/2021.
//

import Foundation
import UIKit

class MainMenuTableController: UITableViewController {
    private let sections =
        [MenuSection(section: .bluetooth, items: [.bleState, .bleStartStop]),
         MenuSection(section: .displaySettings, items: [.settingZoom, .settingLineWidthScale, .settingColorScheme, .settingUpSource]),
         MenuSection(section: .otherSettings, items: [.settingGpsSource, .settingAdditionalData, .settingMapScale])]

    override func viewDidLoad() {
        super.viewDidLoad()
        title = "Main menu"
        BLEBikeAccessory.instance?.delegate = self
    }
}

// MARK: - UITableViewDataSource
extension MainMenuTableController {
    override func numberOfSections(in tableView: UITableView) -> Int {
        sections.count
    }

    override func tableView(_ tableView: UITableView, numberOfRowsInSection sectionIndex: Int) -> Int {
        sections[sectionIndex].items.count
    }

    override func tableView(_ tableView: UITableView, cellForRowAt indexPath: IndexPath) -> UITableViewCell {
        let defaultCellIdentifier = "MainMenuDefaultCell"
        let cell = { () -> UITableViewCell in
            if let cell = tableView.dequeueReusableCell(withIdentifier: defaultCellIdentifier) {
                return cell
            } else {
                return UITableViewCell(style: .value1, reuseIdentifier: defaultCellIdentifier)
            }
        }()

        let menuItem = menuItemForIndexPath(indexPath)
        cell.textLabel?.text = menuItem.title
        cell.detailTextLabel?.text = valueForMenuItem(menuItem)
        return cell
    }
}

// MARK: - UITableViewDelegate
extension MainMenuTableController {
    override func tableView(_ tableView: UITableView, viewForFooterInSection section: Int) -> UIView? {
        return UIView()
    }

    override func tableView(_ tableView: UITableView, heightForFooterInSection section: Int) -> CGFloat {
        return 20
    }

    override func tableView(_ tableView: UITableView, titleForHeaderInSection sectionIndex: Int) -> String? {
        sections[sectionIndex].section.title
    }

    override func tableView(_ tableView: UITableView, didSelectRowAt indexPath: IndexPath) {
        tableView.deselectRow(at: indexPath, animated: true)
        let menuItem = menuItemForIndexPath(indexPath)
        handleTapOnMenuItem(menuItem)
    }
}

// MARK: - BLEBikeDelegate
extension MainMenuTableController: BLEBikeDelegate {
    func stateDidChange(_ newState: BLEBikeAccessoryState) {
        reloadSection(.bluetooth)
    }

    func updateRequested() {
    }
}

// MARK: - private: menu content
private extension MainMenuTableController {
    enum Section {
        case bluetooth
        case displaySettings
        case otherSettings

        var title: String {
            switch self {
            case .bluetooth:
                return "Bluetooth Low Energy"
            case .displaySettings:
                return "Display settings"
            case .otherSettings:
                return "Other settings"
            }
        }
    }

    enum MenuItem {
        case bleState
        case bleStartStop
        case settingZoom
        case settingUpSource
        case settingColorScheme
        case settingLineWidthScale
        case settingGpsSource
        case settingAdditionalData
        case settingMapScale

        var title: String {
            switch self {
            case .bleState:
                return "State"
            case .bleStartStop:
                return "START/STOP"
            case .settingZoom:
                return "Meters per pixel"
            case .settingUpSource:
                return "Display top"
            case .settingColorScheme:
                return "Color scheme"
            case .settingLineWidthScale:
                return "Line scale"
            case .settingGpsSource:
                return "GPS source"
            case .settingAdditionalData:
                return "Additional data"
            case .settingMapScale:
                return "Map scale"
            }
        }
    }

    struct MenuSection {
        let section: Section
        let items: [MenuItem]
    }

    func reloadSection(_ section: Section) {
        guard let index = sections.firstIndex(where: { $0.section == section }) else {
            return
        }
        tableView.reloadSections([index], with: .none)
    }

    func menuItemForIndexPath(_ indexPath: IndexPath) -> MenuItem {
        sections[indexPath.section].items[indexPath.row]
    }

    func valueForMenuItem(_ item: MenuItem) -> String? {
        switch item {
        case .bleState:
            return BLEBikeAccessory.instance?.state.humanReadableString ?? "unknown"
        case .bleStartStop:
            return BLEBikeAccessory.instance == nil ? "Stopped" : "Running"
        case .settingZoom:
            guard let zoom = Settings.instance?.metersPerPixel else {
                return "Error"
            }
            return "\(zoom)"
        case .settingUpSource:
            guard let upSource = Settings.instance?.upSource else {
                return "Error"
            }
            switch upSource {
            case .northUp:
                return "North"
            case .systemCourseUp:
                return "GPS course (system)"
            case .customCourseUp:
                return "Course (computed)"
            case .headingUp:
                return "Heading"
            }
        case .settingColorScheme:
            guard let colorScheme = Settings.instance?.colorScheme else {
                return "Error"
            }
            switch colorScheme {
            case .dark:
                return "Dark"
            case .light:
                return "Light"
            }
        case .settingLineWidthScale:
            guard let scale = Settings.instance?.lineWidthScale else {
                return "Error"
            }
            return "\(scale)"
        case .settingGpsSource:
            guard let positionSource = Settings.instance?.positionSource else {
                return "Error"
            }
            switch positionSource {
            case .real:
                return "Real GPS"
            case .mapViewCenter:
                return "Center of map view"
            }
        case .settingAdditionalData:
            guard let additionalData = Settings.instance?.additionalData else {
                return "Error"
            }
            switch additionalData {
            case .none:
                return "None"
            case .time:
                return "Time"
            }
        case .settingMapScale:
            guard let mapScale = Settings.instance?.mapScale else {
                return "Error"
            }
            switch mapScale {
            case .tile256:
                return "1x"
            case .tile512:
                return "2x"
            }
        }
    }

    func handleTapOnMenuItem(_ item: MenuItem) {
        switch item {
        case .bleState:
            reloadSection(.bluetooth)
        case .bleStartStop:
            if BLEBikeAccessory.instance == nil {
                BLEBikeAccessory.createAndStart()
                BLEBikeAccessory.instance?.delegate = self
            } else {
                BLEBikeAccessory.stopAndDestroy()
            }
            reloadSection(.bluetooth)
        case .settingZoom:
            Settings.instance?.switchMetersPerPixel()
            reloadSection(.displaySettings)
        case .settingUpSource:
            Settings.instance?.switchUpSource()
            reloadSection(.displaySettings)
        case .settingColorScheme:
            Settings.instance?.switchColorScheme()
            reloadSection(.displaySettings)
        case .settingLineWidthScale:
            Settings.instance?.switchLineWidthScale()
            reloadSection(.displaySettings)
        case .settingGpsSource:
            Settings.instance?.switchPositionSource()
            reloadSection(.otherSettings)
        case .settingAdditionalData:
            Settings.instance?.switchAdditionalData()
            reloadSection(.otherSettings)
        case .settingMapScale:
            Settings.instance?.switchMapScale()
            reloadSection(.otherSettings)
        }
    }
}
