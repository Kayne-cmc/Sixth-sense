from enum import Enum

TOTAL_MOTORS = 4
MAX_DISTANCE = 4
MAX_STRENGTH = 100
MIN_STRENGTH = 50

# Patterned vibrations
ALERT_DISTANCE = 0.7
Patterns = Enum('Patterns', ['PULSE', 'DOUBLE_TAP'])
PULSE_DURATION = 0.35
DAMPEN_COEFFICIENT = 0.6 # Dampen stength of other motors when one is pulsing

#Benchmarking
TEST_TITLES = {
    'startup': "Startup Time",
    'response': "Response Time"
}