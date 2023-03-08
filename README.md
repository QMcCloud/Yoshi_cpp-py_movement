> a README file (PDFs welcome!) describing your approach in reactor to analyzing /ir_intensity and the "control" scheme that you used to steer it 


A quick descriptio of the [Reactor member function](https://github.com/QMcCloud/Yoshi_cpp-py_movement/blob/main/reactor_member_function.py)
We first make a [subscriber](https://github.com/QMcCloud/Yoshi_cpp-py_movement/blob/main/reactor_member_function.py#L30) to listen to the IR intensity topic with a callback function of [ir_callback](https://github.com/QMcCloud/Yoshi_cpp-py_movement/blob/main/reactor_member_function.py#L51).

We also make a [publisher](https://github.com/QMcCloud/Yoshi_cpp-py_movement/blob/main/reactor_member_function.py#L37) to send a twist message to the cmd vel topic.
The [pub_cmd_vel_callback](https://github.com/QMcCloud/Yoshi_cpp-py_movement/blob/main/reactor_member_function.py#L48) is called by the [timer](https://github.com/QMcCloud/Yoshi_cpp-py_movement/blob/main/reactor_member_function.py#L40) to publish the robot's twist command every 0.25 seconds.

In order to analyze the ir intensity values, we [map](https://github.com/QMcCloud/Yoshi_cpp-py_movement/blob/main/reactor_member_function.py#L52) them into a list.

We then sum the [left](https://github.com/QMcCloud/Yoshi_cpp-py_movement/blob/main/reactor_member_function.py#L56) and [right](https://github.com/QMcCloud/Yoshi_cpp-py_movement/blob/main/reactor_member_function.py#L57) sides of ir intensity values.

we [compare](https://github.com/QMcCloud/Yoshi_cpp-py_movement/blob/main/reactor_member_function.py#L58) the left and right values to determine if the robot should turn [right](https://github.com/QMcCloud/Yoshi_cpp-py_movement/blob/main/reactor_member_function.py#L59) or [left](https://github.com/QMcCloud/Yoshi_cpp-py_movement/blob/main/reactor_member_function.py#L61).

the turn rate is then [scaled](https://github.com/QMcCloud/Yoshi_cpp-py_movement/blob/main/reactor_member_function.py#L64) down and squished with a [sigmoid](https://github.com/QMcCloud/Yoshi_cpp-py_movement/blob/main/reactor_member_function.py#L67) function.

With the turn rate now within the range &plusmn;1 we use it to determine the [forward rate](https://github.com/QMcCloud/Yoshi_cpp-py_movement/blob/main/reactor_member_function.py#L70) such that the turn and forward rates form a pseudo unit vector.

Lastly we set the robot's twist [turn rate](https://github.com/QMcCloud/Yoshi_cpp-py_movement/blob/main/reactor_member_function.py#L72) and [forward rate](https://github.com/QMcCloud/Yoshi_cpp-py_movement/blob/main/reactor_member_function.py#L73) rescaled up to the max allowed values, and [log](https://github.com/QMcCloud/Yoshi_cpp-py_movement/blob/main/reactor_member_function.py#L75) the values and twist.
