from ros_computer_setup.menu import Menu, MenuEntry, HelpMenu
from ros_computer_setup.netplan import Netplan


class RobotComputerSetup():
    title = r"""
  ___     _         _      ___                     _             ___      _             
 | _ \___| |__  ___| |_   / __|___ _ __  _ __ _  _| |_ ___ _ _  / __| ___| |_ _  _ _ __ 
 |   / _ \ '_ \/ _ \  _| | (__/ _ \ '  \| '_ \ || |  _/ -_) '_| \__ \/ -_)  _| || | '_ \
 |_|_\___/_.__/\___/\__|  \___\___/_|_|_| .__/\_,_|\__\___|_|   |___/\___|\__|\_,_| .__/
                                        |_|                                       |_|
"""

    def __init__(self) -> None:
        self.netplan = Netplan()
        self.entries = [MenuEntry(entry='Netplan', function=self.netplan.run),
                        MenuEntry(entry='About', function=self.about),
                        MenuEntry(entry='Exit', function=self.exit)]
        self.menu = Menu(self.title, self.entries)

    def run(self):
        self.menu.show()

    def exit(self):
        pass

    def about(self):
        about = """
ROS Computer Setup.
"""
        HelpMenu(about, display_help_title=False).show()

def main():
    configurator = RobotComputerSetup()
    configurator.run()

if __name__ == '__main__':
    main()