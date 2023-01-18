### Data Analysis by rosbag replay

#### Replay the state machine only:

- `use_sim_time` should be true
- add `--clock` in rosbag play
- rosbag can be selected from [google drive folder for kashiwa 2020.09.27](https://drive.google.com/drive/folders/1vEDaLRPRugQ-FfyeGaE9L8GWeG8jxV0G?usp=share_link)
- replay state machine:  
  ```bash
  $ roslaunch mbzirc2020_task2_common st_replay.launch
  ```
