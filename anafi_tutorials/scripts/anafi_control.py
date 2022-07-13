#!/usr/bin/env python3
import rospy

from std_msgs.msg import Int16


class anafi():
    def __init__(self):
        self.status = False
        self.pub_takeoff = rospy.Publisher(
            "/anafi/takeoff", Int16, queue_size=1)
        self.pub_land = rospy.Publisher("/anafi/land", Int16, queue_size=1)
        self.user = Int16()

    def main(self):
        self.user.data = int(input("Enter 1 or 2: "))
        print(self.user.data)
        if self.user.data == 1 and not self.status:
            self.pub_takeoff.publish(self.user.data)
            rospy.loginfo('Takeoff')
            self.status = True

        if self.user.data == 2 and self.status:
            self.pub_land.publish(self.user.data)
            rospy.loginfo('Land')
            self.status = False


if __name__ == '__main__':
    rospy.init_node("publisher_node")
    rospy.loginfo('Publisher initialized')
    rate = rospy.Rate(10)
    app = anafi()
    while not rospy.is_shutdown():
        try:
            app.main()
            rate.sleep()
        except KeyboardInterrupt:
            quit()
