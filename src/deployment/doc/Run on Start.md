# Run on Start

- Copy the python file to `~/.local/bin` and make it executable:

  ```
  cp -i /path/to/device.py $HOME/.local/bin
  chmod u+x $HOME/.local/bin/device.py
  ```

- Add A New Cron Job:

  `crontab -e`

  Scroll to the bottom and add the following line (after all the `#'s`):

  `@reboot $HOME/.local/bin/device.py &`

  The "&" at the end of the line means the command is run in the background and it won't stop the system booting up.

- Test it:

  `sudo reboot`

