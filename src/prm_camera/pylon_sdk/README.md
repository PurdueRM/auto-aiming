# pylon SDK

The Basler pylon Camera Software Suite is not redistributable and sits behind a
free myBasler account, so it is not committed here.

1. Create an account at https://www.baslerweb.com and go to Downloads -> Software -> pylon Camera Software Suite.
2. Filter the OS to **Linux ARM 64 bit (aarch64)**. pylon 7.5 or 8.x both work with the humble driver.
3. Drop the downloaded archive in this directory.
4. Run `./install_pylon.sh` (installs to /opt/pylon, needs sudo), or `./install_pylon.sh --local` to install into ./pylon with no sudo.
