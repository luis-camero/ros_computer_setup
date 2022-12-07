from ros_computer_setup.menu import Menu, MenuEntry, OptionsMenu, Prompt, list_files

from netplan.netplan import NetplanConfig, NetplanBridgeConfig, NetplanEthernetConfig, NetplanWifiConfig

BRIDGE_TITLE = r"""
  ___     _    _            ___      _             
 | _ )_ _(_)__| |__ _ ___  / __| ___| |_ _  _ _ __ 
 | _ \ '_| / _` / _` / -_) \__ \/ -_)  _| || | '_ \
 |___/_| |_\__,_\__, \___| |___/\___|\__|\_,_| .__/
                |___/                        |_|   
"""
ETHERNET_TITLE = r"""
  ___ _   _                     _     ___      _             
 | __| |_| |_  ___ _ _ _ _  ___| |_  / __| ___| |_ _  _ _ __ 
 | _||  _| ' \/ -_) '_| ' \/ -_)  _| \__ \/ -_)  _| || | '_ \
 |___|\__|_||_\___|_| |_||_\___|\__| |___/\___|\__|\_,_| .__/
                                                       |_|   
"""
NETPLAN_TITLE = r"""
  _  _     _        _              ___           __ _                    _   _          
 | \| |___| |_ _ __| |__ _ _ _    / __|___ _ _  / _(_)__ _ _  _ _ _ __ _| |_(_)___ _ _  
 | .` / -_)  _| '_ \ / _` | ' \  | (__/ _ \ ' \|  _| / _` | || | '_/ _` |  _| / _ \ ' \ 
 |_|\_\___|\__| .__/_\__,_|_||_|  \___\___/_||_|_| |_\__, |\_,_|_| \__,_|\__|_\___/_||_|
              |_|                                    |___/                              
"""
WIFI_TITLE = r"""
 __      ___ ___ _   ___      _              
 \ \    / (_) __(_) / __| ___| |_ _  _ _ __  
  \ \/\/ /| | _|| | \__ \/ -_)  _| || | '_ \ 
   \_/\_/ |_|_| |_| |___/\___|\__|\_,_| .__/ 
                                      |_|    
"""

class BridgeSelection():
    title = BRIDGE_TITLE

class BridgeSetup():
    title = BRIDGE_TITLE

class EthernetSetup():
    title = ETHERNET_TITLE

class WifiSetup():
    title = WIFI_TITLE

    def __init__(self, configs: NetplanWifiConfig) -> None:
        self.configs = configs

        self.entries = [MenuEntry(entry=lambda: 'SSID: {0}'.format(self.configs.ssid), function=self.set_ssid),
                        MenuEntry(entry=lambda: 'Password: {0}'.format(self.configs.password),function=self.set_password),
                        #MenuEntry(entry=lambda: 'Wi-Fi Mode: {0}'.format(self.configs.wifi_mode),function=self.set_wifi_mode),
                        #MenuEntry(entry=lambda: 'Regulatory Domain: {0}'.format(self.configs.reg_domain),function=self.set_reg_domain),
                        #MenuEntry(entry=lambda: 'Band: {0}'.format(self.configs.band),function=self.set_band),
                        #MenuEntry(entry=lambda: 'IP Address: {0}'.format(self.configs.ip_address),function=self.set_ip_address),
                        MenuEntry('', None),
                        MenuEntry(entry='Save Settings', function=self.save_netplan_settings),
                        MenuEntry(entry='Reset Settings', function=self.configs.read)]

        self.menu = Menu(self.title, self.entries)

    def run(self):
        #self.configs.read()
        self.menu.show()

    def set_ssid(self):
        p = Prompt(prompt='SSID ({0}): '.format(self.configs.ssid),
                   default_response=self.configs.ssid,
                   note='Wi-Fi Network SSID')
        self.configs.ssid = p.show()

    def set_password(self):
        p = Prompt(prompt='Password ({0}): '.format(self.configs.password),
                   default_response=self.configs.password,
                   note='Wi-Fi Network Password')
        self.configs.password = p.show()

    def set_reg_domain(self):
        p = Prompt(prompt='Regulatory Domain ({0}): '.format(self.configs.reg_domain),
                   default_response=self.configs.reg_domain,
                   note='Wireless regulatory domain. \n' +
                        'Common options:\n' +
                        'USA: US\nCanada: CA\nUK: GB\n' +
                        'Germany: DE\nJapan: JP3\nSpain: ES')
        self.configs.reg_domain = p.show()

    def set_wifi_mode(self):
        options = OptionsMenu(title='Wi-Fi Mode',
                              menu_entries=['Client', 'Access Point'],
                              default_option=self.configs.wifi_mode)
        self.configs.wifi_mode = options.show()

    def set_band(self):
        options = OptionsMenu(title='Band',
                              menu_entries=['5GHz', '2.4GHz', 'Any'],
                              default_option=self.configs.band)
        self.configs.band = options.show()

    def set_ip_address(self):
        p = Prompt(prompt='IP Address ({0}): '.format(self.configs.ip_address),
                   default_response=self.configs.ip_address,
                   note='IP Address with CIDR. e.g. 192.168.0.1/24')
        self.configs.ip_address = p.show()

    def save_netplan_settings(self):
        #self.configs.write()
        self.menu.exit()

class WifiMenu():
    title = WIFI_TITLE

    def __init__(self) -> None:
        pass

class Netplan():
    title = NETPLAN_TITLE

    def __init__(self) -> None:
        # Netplan Configuration Module
        self.configs = NetplanConfig()
        self.configs.load()
        # Bridge Setup
        # Ethernet Setup
        # Wifi Setup
        self.wifi_setup = WifiSetup(self.configs.wifis[0])
        # View Netplan Configs
        self.entries = [MenuEntry("Wi-Fi Setup", self.wifi_setup.run)]
        self.menu = Menu(self.title, self.entries)

    def run(self):
        self.menu.show()
