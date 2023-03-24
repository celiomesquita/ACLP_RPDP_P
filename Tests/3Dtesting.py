from py3Druiz import Packer as pr
from py3Druiz import Bin as br
from py3Druiz import Item as ir

from py3Djanet.packer import Packer as pj
from py3Djanet.methods import Item as ij
from py3Djanet.bin import Bin as bj

import random
import time

if __name__ == "__main__":

    packerR = pr()
    packerR.add_bin(br('palletR', 215, 330, 265, 4500, 1))

    packerJ = pj()
    packerJ.add_bin(bj('palletJ', 215, 330, 265, 4500, 2))


    n = 220

    for id in range (n):

        name = f'item{id}'
        width  = random.uniform(20, 60)
        height = random.uniform(20, 60)
        depth  = random.uniform(30, 80)
        weight = random.uniform(10, 30)

        # 12000 to 288000

        packerR.add_item(ir( name, width, height, depth, weight, id))
        packerJ.add_item(ij( name, width, height, depth, weight, id))


    start = time.perf_counter()

    packerR.pack()
    for b in packerR.bins:
        print(f"R fit {len(b.items)} | unfit {len(b.unfitted_items)} | {100*len(b.unfitted_items)/(len(b.items) + len(b.unfitted_items)):.1f}%")
        print("R filling ratio: ", b.get_filling_ratio())

    print(f"R time {time.perf_counter()-start:.2f} s")

    start = time.perf_counter()

    packerJ.pack()
    for b in packerJ.bins:
        print(f"J fit {len(b.items)} | unfit {len(b.unfitted_items)} | {100*len(b.unfitted_items)/(len(b.items) + len(b.unfitted_items)):.1f}%")
        print("J filling ratio: ", b.get_filling_ratio())

    print(f"J time {time.perf_counter()-start:.2f} s")
 