#!/usr/bin/python
from client import setup_client
from helpers.movements import calibrate_zero
from helpers.math_tools import estimate_jacobian

def main():
    client = setup_client()
    calibrate_zero()
    Jacobian = estimate_jacobian(client, 10, 10)

    print("\n\nJacobian:")
    print(Jacobian)

if __name__ == "__main__":
    main()
