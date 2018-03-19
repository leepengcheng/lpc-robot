import numpy as np

d, theta, a, alpha = 'd', 'theta', 'a', 'alpha'

DH = [
    {d: 0.0661,
     theta: 0.0,
     a: 0.0050,
     alpha: 90.0},
    {d: -0.0000,
     theta: 90.0,
     a: 0.4251,
     alpha: -90.0},
    {d: 0.0000,
     theta: 0.0,
     a: 0.3922,
     alpha: 0.0},
    {d: 0.1150,
     theta: -180.0,
     a: 0.0059,
     alpha: -90.0},
    {d: 0.0000,
     theta: -180.0,
     a: 0.1007,
     alpha: -90.0}
]
import argparse
parser=argparse.ArgumentParser(description="my argument")
parser.add_argument("--name",default="leepengcheng",type=str,help="your name")
parser.add_argument('--age', default=0.6, type=int,help='your age')
args=parser.parse_args()
print(args.name)