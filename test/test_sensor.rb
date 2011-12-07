require 'orocos/test/component'
require 'test/unit'

class TC_Sensor < Test::Unit::TestCase
    include Orocos::Test::Component

    start 'task', 'tut_sensor::Task'
    writer 'task', 'local_frame'
    writer 'task', 'target_frame'
    reader 'task', 'target_sensor_sample'

    def test_circle_backward
        # The rock can return a "returned" orientation. I.e. the Z axis of the
        # local frame points down, which means that the actual direction of
        # travel is -X, not X
        #
        # This unit test tests this
        test_circle(Math::PI)
    end

    def test_circle(base_pitch = 0)
        task.start

        local = Types::Base::Samples::RigidBodyState.new
        local.zero!
        local.position = Eigen::Vector3.new(0, 0, 0)
        local.orientation = Eigen::Quaternion.from_angle_axis(base_pitch, Eigen::Vector3.UnitY())
        local.time = Time.now
        task_local_frame.write(local)

        target = Types::Base::Samples::RigidBodyState.new
        target.zero!

        step_size = Math::PI / 6
        (-5..5).each do |i|
            angle = step_size * i
            target.time = Time.now
            target.position = Eigen::Vector3.new(Math.cos(angle), Math.sin(angle), 0)
            target.orientation = Eigen::Quaternion.Identity
            task_target_frame.write(target)

            sample = assert_has_one_new_sample(task_target_sensor_sample)
            assert_in_delta angle, sample.bearing.rad, 0.01
            assert_in_delta 1, sample.distance, 0.01
        end
    end
end

