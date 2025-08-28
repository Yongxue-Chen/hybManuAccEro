def caliByCutting(x1, y1, x2, y2, x3, y3, x4, y4):
    """
    Compute origin offsets for AM and SM from cutting deviation measurements.

    Parameters:
    - x1, y1, x2, y2, x3, y3, x4, y4:
      Deviation measurements (sm - am) in X and Y under four horizontal
      cutting directions.

    Direction order and mapping:
    1) From negative Y direction (Y-): (x1, y1)
    2) From positive Y direction (Y+): (x2, y2)
    3) From negative X direction (X-): (x3, y3)
    4) From positive X direction (X+): (x4, y4)

    Returns:
    - DxA, DyA: Origin offsets of AM in X and Y.
    - DxS, DyS: Origin offsets of SM in X and Y.

    Notes:
    - Inputs represent the relative position difference: sm - am.
    - The function solves the linear system (possibly noisy) via least squares.
      Using the equations:
        DxS - DxA = x1;  DyS - DyA = y1
        DxS + DxA = x2;  DyS + DyA = y2
        DxS + DyA = x3;  DyS - DxA = y3
        DxS - DyA = x4;  DyS + DxA = y4
      The closed-form least-squares solution is:
        DxA = (-x1 + x2 - y3 + y4) / 4
        DyA = (-y1 + y2 + x3 - x4) / 4
        DxS = (x1 + x2 + x3 + x4) / 4
        DyS = (y1 + y2 + y3 + y4) / 4
    """
    DxA = (-x1 + x2 - y3 + y4) / 4.0
    DyA = (-y1 + y2 + x3 - x4) / 4.0
    DxS = (x1 + x2 + x3 + x4) / 4.0
    DyS = (y1 + y2 + y3 + y4) / 4.0
    return DxA, DyA, DxS, DyS

def caliByCutting_partial_x1y1x4y4(x1, y1, x4, y4):
    """
    Solve origin offsets using only (x1, y1, x4, y4).

    Equations used:
      DxS - DxA = x1;  DyS - DyA = y1
      DxS - DyA = x4;  DyS + DxA = y4

    Closed-form solution:
      DxA = ((x4 - x1) + (y4 - y1)) / 2
      DyA = ((y4 - y1) - (x4 - x1)) / 2
      DxS = x1 + DxA
      DyS = y1 + DyA

    Returns:
    - DxA, DyA, DxS, DyS
    """
    DxA = ((x4 - x1) + (y4 - y1)) / 2.0
    DyA = ((y4 - y1) - (x4 - x1)) / 2.0
    DxS = x1 + DxA
    DyS = y1 + DyA
    return DxA, DyA, DxS, DyS

def caliByCutting_partial_x1y1x2y2x4y4(x1, y1, x2, y2, x4, y4):
    """
    Solve origin offsets using (x1, y1, x2, y2, x4, y4) via least squares.

    Equations used:
      DxS - DxA = x1
      DyS - DyA = y1
      DxS + DxA = x2
      DyS + DyA = y2
      DxS - DyA = x4
      DyS + DxA = y4

    Closed-form least-squares solution (normal equations):
      DxA = (-3*x1 + 3*x2 - y1 - y2 + 2*y4) / 8
      DyA = ( x1 + x2 - 2*x4 - 3*y1 + 3*y2) / 8
      DxS = ( 3*x1 + 3*x2 + 2*x4 - y1 + y2) / 8
      DyS = ( x1 - x2 + 3*y1 + 3*y2 + 2*y4) / 8
    """
    DxA = (-3*x1 + 3*x2 - y1 - y2 + 2*y4) / 8.0
    DyA = ( x1 + x2 - 2*x4 - 3*y1 + 3*y2) / 8.0
    DxS = ( 3*x1 + 3*x2 + 2*x4 - y1 + y2) / 8.0
    DyS = ( x1 - x2 + 3*y1 + 3*y2 + 2*y4) / 8.0
    return DxA, DyA, DxS, DyS

if __name__ == "__main__":
    x1, y1, x2, y2, x3, y3, x4, y4 = 0, 0, 1.2, -0.2, 0.0, 0.0, 0.4, -0.2
    # DxA, DyA, DxS, DyS = caliByCutting(x1, y1, x2, y2, x3, y3, x4, y4)
    # print(f"DxA: {DxA}, DyA: {DyA}, DxS: {DxS}, DyS: {DyS}")
    # DxA, DyA, DxS, DyS = caliByCutting_partial_x1y1x4y4(x1, y1, x4, y4)
    DxA, DyA, DxS, DyS = caliByCutting_partial_x1y1x2y2x4y4(x1, y1, x2, y2, x4, y4)
    print(f"DxA: {DxA}, DyA: {DyA}, DxS: {DxS}, DyS: {DyS}")