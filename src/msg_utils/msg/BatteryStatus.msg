std_msgs/Header header         # Metadata for timestamping and frame ID.
float64 voltage        # Main voltage of the battery in volts.
float64 current         # Current being drawn from the battery in amperes.
float64 charge_level    #  Current charge level of the battery as a percentage (0-100%).
bool charging           # Whether the battery is currently being charged (true/false)