#!/usr/bin/python
from Client import setup_client
from helpers.movements import calibrate_zero
from helpers.math_tools import estimate_jacobian

def main():
    client = setup_client()
    calibrate_zero()
    Jacobian = estimate_jacobian(client, 10, 10)


if __name__ == "__main__":
    main()
