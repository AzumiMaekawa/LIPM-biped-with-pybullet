# to calculate inertia of primitive shapes
import argparse

def sphere_inertia():
    print('input: mass radius')
    m ,r = [float(param) for param in input().split()]
    print("ixx={0} ixy={1} ixz={1}\niyy={0} iyz={1}\nizz={0}"\
            .format(2*m*r*r/5, 0))
    return

def cylinder_inertia():
    print('input: mass radius height')
    m ,r, h = [float(param) for param in input().split()]
    print("ixx={0} ixy={1} ixz={1}\niyy={0} iyz={1}\nizz={2}"\
            .format(m*(3*r*r+h*h)/12, 0, m*r*r/2))

def box_inertia():
    print('input: mass x y z')
    m ,x, y, z = [float(param) for param in input().split()]
    print("ixx={0} ixy={1} ixz={1}\niyy={2} iyz={1}\nizz={3}"\
            .format(m*(y*y+z*z)/12, 0, m*(x*x+z*z)/12, m*(x*x+z*z)/12))

def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("geometry", type=str, help="set geometry type(s: sphere, \
            c: cylinder, b: box")
    args = parser.parse_args()
    if args.geometry=='s': # sphere
        sphere_inertia()
    if args.geometry=='c': # cylinder
        cylinder_inertia()
    if args.geometry=='b': # box
        box_inertia()
    
    
if __name__ == "__main__":
    main()
        
    
