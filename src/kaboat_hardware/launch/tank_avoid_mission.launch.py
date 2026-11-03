"""Run the production obstacle avoidance mission alone in the indoor tank."""

from kaboat_hardware.tank_mission_launch import generate_tank_mission_launch


def generate_launch_description():
    return generate_tank_mission_launch('avoid')
