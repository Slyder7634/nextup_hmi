# Installing Nextup Cobot HMI on another PC

Assumes ROS2 and the robot workspaces (`~/nextup_controllers`, `~/x_cat`,
`~/NextupRobot`) are already set up on the target PC.

## 1. Get the package there

Copy `nextup_hmi_package.tar.gz` over — USB stick, `scp`, shared drive,
whatever's easiest:

```bash
scp nextup_hmi_package.tar.gz user@other-pc:~/
```

## 2. Extract and install

On the other PC:

```bash
tar xzf nextup_hmi_package.tar.gz
cd nextup_hmi_package
./install.sh
```

This copies everything (code, URDF, meshes) to `~/.local/share/nextup_hmi`,
and creates a `nextup-hmi` command.

## 3. If it warns about PATH

If you see a note that `~/.local/bin` isn't on your PATH, run:

```bash
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

## 4. Run it

```bash
nextup-hmi
```

That's it — it'll source ROS2 + your three workspaces, launch the MoveIt
stack, start the servo node, and open the HMI window. You can also just
search "Nextup Cobot HMI" in the applications menu since the installer
added a launcher entry there too.

## If something goes wrong

`run.sh` checks for each ROS2 setup file before sourcing it, so if a
workspace path is off, you'll get a clear error naming the missing file
instead of a silent failure — worth checking that `~/nextup_controllers`,
`~/x_cat`, and `~/NextupRobot` exist at those exact paths on this PC (same
as the original dev machine).

## Updating later

Re-run `./install.sh` from a newer copy of the package — it just
overwrites `~/.local/share/nextup_hmi`.

## Uninstall

```bash
rm -rf ~/.local/share/nextup_hmi
rm ~/.local/bin/nextup-hmi
rm ~/.local/share/applications/nextup-hmi.desktop
```
