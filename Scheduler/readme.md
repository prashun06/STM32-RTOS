Must use time_slicing 1/0.

Time will sliced in between all the task.

In course the task execute in different time .....but in code all task run at the same time even in different task priority

remove the osDelay and use for pdMS to make a non blocking loop for the accurate time slicing.
