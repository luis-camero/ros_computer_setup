import os
import psutil
import yaml

class NetplanBridgeConfig():

    def __init__(self, interface, addresses=[], bridged=[], dhcp4="yes", dhcp6="no") -> None:
        self.interface = interface
        self.addresses = addresses
        self.bridged = bridged
        self.dhcp4 = dhcp4
        self.dhcp6 = dhcp6

class NetplanEthernetConfig():

    def __init__(self, interface, addresses=[], dhcp4="yes", dhcp6="no") -> None:
        self.interface = interface
        self.addresses = addresses
        self.dhcp4 = dhcp4
        self.dhcp6 = dhcp6

    def read(self, path):
        netplan = yaml.load(open(path), yaml.SafeLoader)
        config = netplan['network']['ethernets'][self.interface]
        # Get Addresses and DHCP
        self.addresses = config["addresses"]
        self.dhcp4 = config["dhcp4"]
        self.dhcp6 = config["dhcp6"]

class NetplanWifiConfig():

    def __init__(self, interface, ssid="ssid", password="password") -> None:
        self.interface = interface
        self.ssid = ssid
        self.password = password

    def read(self, path):
        netplan = yaml.load(open(path), yaml.SafeLoader)
        config = netplan['network']['wifis'][self.interface]
        # Get SSID and Password
        self.ssid = list(config['access-points'])[0]
        ssid_settings = config['access-points'][self.ssid]
        self.password = ssid_settings['password']

        # DHCP4
        # Access-Point vs Client
        # Band?
        # Routes?
        # Static Addresses?
        # Namespace Addresses?

class NetplanConfig():

    def __init__(self) -> None:
        self.ethernets = []
        self.wifis = []
        self.bridges = []
        self.discarded = []

    # load : find 
    def load(self):
        if_addrs = psutil.net_if_addrs()
        # Find all present interfaces
        interfaces = [interface for interface in if_addrs]
        bridges =   [interface for interface in interfaces if interface[0] == 'b']
        ethernets = [interface for interface in interfaces if interface[0] == 'e']
        wifis =     [interface for interface in interfaces if interface[0] == 'w']
        # Load Bridges : Empty Bridge
        for bridge in bridges:
            self.add_bridge(interface=bridge,
                            addresses=[if_addrs[bridge][0].address])
        # Load Ethernets : DHCP4 Enabled Interfaces
        for ethernet in ethernets:
            self.add_ethernet(interface=ethernet)
        # Load Wifis : Dummy Variables Wifi
        for wifi in wifis:
            self.add_wifi(interface=wifi)

    # read : extract netplan configration from netplan YAML file.
    def read(self, path):
        config = yaml.load(open(path), yaml.SafeLoader)
        # Get Ethernets
        if "ethernets" in config:
            for interface in config["ethernets"]:
                ethernet_config = NetplanEthernetConfig(interface)
                ethernet_config.read(path)
                self.ethernets.append(ethernet_config)

        # Get Wifis
        if "wifis" in config:
            for interface in config["wifis"]:
                wifi_config = NetplanWifiConfig(interface)
                wifi_config.read(path)
                self.wifis.append(wifi_config)

        # Get Bridges
        if "bridges" in config:
            for interface in config["bridges"]:
                bridge_config = NetplanBridgeConfig(interface)
                bridge_config.read(path)
                self.bridges.append(bridge_config)

    def add_bridge(self, interface, addresses=[], dhcp4="yes", dhcp6="no"):
        self.bridges.append(NetplanBridgeConfig(interface=interface, addresses=addresses, dhcp4=dhcp4, dhcp6=dhcp6))

    def add_ethernet(self, interface, addresses=[], dhcp4="yes", dhcp6="no"):
        self.ethernets.append(NetplanEthernetConfig(interface=interface, addresses=addresses, dhcp4=dhcp4, dhcp6=dhcp6))

    def add_wifi(self, interface, ssid="ssid", password="password"):
        self.wifis.append(NetplanWifiConfig(interface=interface, ssid=ssid, password=password))
