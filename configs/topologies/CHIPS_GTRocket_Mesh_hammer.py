# Copyright (c) 2010 Advanced Micro Devices, Inc.
#               2016 Georgia Institute of Technology
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Authors: Brad Beckmann
#          Tushar Krishna
#          Srikant Bharadwaj

from m5.params import *
from m5.objects import *

from BaseTopology import SimpleTopology

class CHIPS_GTRocket_Mesh_hammer(SimpleTopology):
    description = 'CHIPS_GTRocket_Mesh_hammer'

    def __init__(self, controllers):
        self.nodes = controllers

    def makeTopology(self, options, network, IntLink, ExtLink, Router):
        nodes = self.nodes

        # default values for link latency and router latency.
        # Can be over-ridden on a per link/router basis
        link_latency = options.link_latency # used by simple and garnet
        #router_latency = options.router_latency # only used by garnet

        #Determine the types of nodes
        cpu_nodes = []
        l2_nodes = []
        mc_nodes = []
        for node in nodes:
            if node.type == 'L1Cache_Controller':
                cpu_nodes.append(node)
            elif node.type == 'L2Cache_Controller':
                l2_nodes.append(node)
            elif node.type == 'Directory_Controller':
                mc_nodes.append(node)

        #===========================================================
        #                Configuration Calculation
        #===========================================================

        #Enable/Disable sni
        #sni = options.sni
        if(options.sni == '1'):
            sni = True
            sni_latency = 2
        else:
            sni = False
            sni_latency = 0

        #Assuming 8 Chiplets
        num_cpu_chiplets = options.num_chiplets
        #Assuming 64 Cores; 8 Chiplets each containing 8 Cores
        num_cpus_per_chiplet = len(cpu_nodes)/num_cpu_chiplets
        #MemoryController Chiplets; Assuming 4
        num_mc_chiplets = options.num_dirs

        #Mesh rows and columns
        #CPUs in a chiplet are being connected using a Full xBar
        #Hence, skipping the rows/columns calculation

        #The following calculation may seem incorrect at first but it
        #is what we want. For Rocket Chip, there will be 64 Routers
        #for each core, 8 Routers in each chiplet, 4 Routers for the
        #MC, and then again 8 routers which are point-to-point
        #connected to the Routers inside Chiplet. These 8 Routers and
        #4 MC Routers reside in the interposer layer.
        num_routers = (len(cpu_nodes) +
                      num_mc_chiplets +
                      num_cpu_chiplets +
                      4 +
                      num_cpu_chiplets)

        ### Create the routers in the mesh
        ##routers = [Router(router_id=i, latency = router_latency) \
        ##    for i in range(num_routers)]
        ##network.routers = routers

        routers = []
        for i in range(num_routers):
            if(i<=63):
                router_latency = 1
            elif(i>=64 and i<=71):
                router_latency = 4
            elif(i>=72 and i <=83):
                router_latency = 4
            elif(i>=84):
                router_latency = 4
            print("Router_id:" + str(i)) 
            routers.extend(Router(router_id=i, latency = router_latency))
        network.routers = routers

        # link counter to set unique link ids
        link_count = 0


        print("=====================================")
        print("SNI option: " + str(options.sni))
        print("SNI enable: " + str(sni))
        print("Number of CPUs: " + str(len(cpu_nodes)))
        print("Number of L2: " + str(len(l2_nodes)))
        print("Number of MCs: " + str(len(mc_nodes)))
        print("Total Routers: " + str(num_routers))
        print("=====================================")

        #============================================================
        #Connect the External Links between Nodes and Routers
        #SerDes = False
        #============================================================
        ext_links = []

        # start from router 0
        router_id = 0

        # Connect each CPU to a unique router
        # Router 0 to 63 connect to Core 0 to 63
        for (i, n) in enumerate(cpu_nodes):
            cpu_link_width = 128
            #print("Router_id:" + str(router_id))
            routers[router_id].width = cpu_link_width
            ext_links.append(ExtLink(link_id=link_count, ext_node=n,
                                    int_node=routers[router_id],
                                    width=cpu_link_width,
                                    sni = False,
                                    int_serdes = False,
                                    latency = 1))
            print_connection("CPU", n.version, "Router", router_id, \
            link_count, link_latency, cpu_link_width)
            link_count += 1
            router_id += 1


        # start from router 0
        router_id = 0

        # Connect the L2 Controllers
        # Connect them from 0 to 63 for hammer protocol
        for (i, n) in enumerate(l2_nodes):
            l2_link_width = 128
            routers[router_id].width = l2_link_width
            ext_links.append(ExtLink(link_id=link_count, ext_node=n,
                                    int_node=routers[router_id],
                                    width=l2_link_width,
                                    sni = False, 
                                    int_serdes = False,
                                    latency = 1))
            print_connection("L2", n.version, "Router", router_id, \
            link_count, link_latency, l2_link_width)
            link_count += 1
            router_id += 1

        # Router 64 to 71 go to the XBAR in chiplets
        # Connect the MC nodes to corner Routers in Interposer
        # Router 84 --- MC 0
        # Router 85 --- MC 1
        # Router 86 --- MC 2
        # Router 87 --- MC 3
        for (i, n) in enumerate(mc_nodes):
            mc_link_width = 128
            if (i==0):
                routers[84].width = mc_link_width
                ext_links.append(ExtLink(link_id=link_count, ext_node=n,
                                    int_node=routers[84],
                                    width=mc_link_width,
                                    sni = False, 
                                    int_serdes = False,
                                    latency = 4))
                print_connection("MC", n.version, "Router", 84,
                link_count, link_latency, mc_link_width)
            elif (i==1):
                routers[85].width = mc_link_width
                ext_links.append(ExtLink(link_id=link_count, ext_node=n,
                                    int_node=routers[85],
                                    width=mc_link_width,
                                    sni = False, 
                                    int_serdes = False,
                                    latency = 4))
                print_connection("MC", n.version, "Router", 85,
                link_count, link_latency, mc_link_width)
            elif (i==2):
                routers[86].width = mc_link_width
                ext_links.append(ExtLink(link_id=link_count, ext_node=n,
                                    int_node=routers[86],
                                    width=mc_link_width,
                                    sni = False, 
                                    int_serdes = False,
                                    latency = 4))
                print_connection("MC", n.version, "Router", 86,
                link_count, link_latency, mc_link_width)
            elif (i==3):
                routers[87].width = mc_link_width
                ext_links.append(ExtLink(link_id=link_count, ext_node=n,
                                    int_node=routers[87],
                                    width=mc_link_width,
                                    sni = False, 
                                    int_serdes = False,
                                    latency = 4))
                print_connection("MC", n.version, "Router", 87,
                link_count, link_latency, mc_link_width)
            link_count += 1


        #assert(router_id==76)

        #All routers except xbar in each chiplet
        #and the main xBar have been connected

        #============================================================
        #Connect the Internal Links between Routers in each Chiplet
        #SerDes = False
        #============================================================
        # Create the mesh links inside each chiplet
        int_links = []

        #There are 8 Cores in each Chiplet. Each Core is connected
        #to a Full System Xbar.
        chiplet_xbar_id = len(cpu_nodes)
        #Idea is the the full systemxbar in each chiplet will have the ids
        #64 to 71
        for cc in xrange(num_cpu_chiplets):
            assert(chiplet_xbar_id>=64 and chiplet_xbar_id<=71)
            #We can assign a high latency for the xbar router
            routers[chiplet_xbar_id].latency = 4
            for cpu in xrange(num_cpus_per_chiplet):
                chiplet_intlink_width = 128
                router_cores = (cc*num_cpus_per_chiplet)+cpu
                int_links.append(IntLink(link_id=link_count,
                                         src_node=routers[router_cores],
                                         dst_node=routers[chiplet_xbar_id],
                                         src_serdes = False,
                                         dst_serdes = False,
                                         sni=False,
                                         width = chiplet_intlink_width,
                                         latency = 1,
                                         weight=1))

                print_connection("Router", \
                get_router_id(routers[router_cores]), \
                "Router", get_router_id(routers[chiplet_xbar_id]),
                link_count, link_latency, chiplet_intlink_width)

                link_count += 1
                int_links.append(IntLink(link_id=link_count,
                                         src_node=routers[chiplet_xbar_id],
                                         dst_node=routers[router_cores],
                                         src_serdes = False,
                                         dst_serdes = False,
                                         sni=False,
                                         width = chiplet_intlink_width,
                                         latency = 1,
                                         weight=1))

                print_connection("Router", \
                get_router_id(routers[chiplet_xbar_id]), \
                "Router", get_router_id(routers[router_cores]),
                link_count, link_latency, chiplet_intlink_width)


                link_count += 1
            #router_id += 1 This doesn't serve anything here
            chiplet_xbar_id += 1

        #===============================================================
        #                 SerDes Connections Interposer
        #===============================================================

        #assert(noc_xbar_id==76)
        #Reset the chiplet Xbar ids
        chiplet_xbar_id = len(cpu_nodes)
        #Now we connect the full xbar of each chiplet to the Routers
        #inside the interposer
        #TODO:Increasing the latency of the NOC xbar
        #routers[noc_xbar_id].latency = 4
        iterp_width = 128

        #This is the interposer router id to which we will connect
        #the


        for cc in xrange(num_cpu_chiplets):
            if (cc == 0):
                interp_router_id = 72
            elif (cc == 1):
                interp_router_id = 75
            elif (cc == 2):
                interp_router_id = 78
            elif (cc == 3):
                interp_router_id = 81
            elif (cc == 4):
                interp_router_id = 74
            elif (cc == 5):
                interp_router_id = 77
            elif (cc == 6):
                interp_router_id = 80
            elif (cc == 7):
                interp_router_id = 83

            #######################################################
            int_links.append(IntLink(link_id=link_count,
                                     src_node=routers[interp_router_id],
                                     dst_node=routers[chiplet_xbar_id],
                                     src_serdes = False,
                                     dst_serdes = False,
                                     sni=False,
                                     width = iterp_width,
                                     latency = 4,
                                     weight=1))
            print_connection("Router",
            get_router_id(routers[interp_router_id]),
            "Router", get_router_id(routers[chiplet_xbar_id]),
            link_count, link_latency, iterp_width)

            link_count += 1
             
            ########################################################
            int_links.append(IntLink(link_id=link_count,
                                 src_node=routers[chiplet_xbar_id],
                                 dst_node=routers[interp_router_id],
                                 src_serdes = False,
                                 dst_serdes = False,
                                 sni=sni,
                                 width = iterp_width,
                                 latency = 4+sni_latency,
                                 weight=1))
            print("SNI Enabled Connection")
            print_connection("Router", get_router_id(routers[chiplet_xbar_id]),     
            "Router", get_router_id(routers[interp_router_id]),
            link_count, link_latency+5, iterp_width)

            link_count += 1
            ########################################################
            
            chiplet_xbar_id += 1


        interp_router_offset = len(cpu_nodes) + num_cpu_chiplets
        # East output to West input links (weight = 1)
        for row in range(4):
            for col in range(3):
                if (col + 1 < 3):
                    east_out = interp_router_offset + col + (row * 3)
                    west_in = interp_router_offset + (col + 1) + (row * 3)
                    int_links.append(IntLink(link_id=link_count,
                                             src_node=routers[east_out],
                                             dst_node=routers[west_in],
                                             src_serdes = False,
                                             dst_serdes = False,
                                             src_outport="East",
                                             dst_inport="West",
                                             sni=False,
                                             width = iterp_width,
                                             latency = 4,
                                             weight=1))
                    print_connection("Router",
                    get_router_id(routers[east_out]),
                    "Router", get_router_id(routers[west_in]),
                    link_count, link_latency, iterp_width)

                    link_count += 1

        # West output to East input links (weight = 1)
        for row in range(4):
            for col in range(3):
                if (col + 1 < 3):
                    east_in = interp_router_offset + col + (row * 3)
                    west_out = interp_router_offset + (col + 1) + (row * 3)
                    int_links.append(IntLink(link_id=link_count,
                                             src_node=routers[west_out],
                                             dst_node=routers[east_in],
                                             src_serdes = False,
                                             dst_serdes = False,
                                             src_outport="West",
                                             dst_inport="East",
                                             sni=False,
                                             width = iterp_width,
                                             latency = 4,
                                             weight=1))
                    print_connection("Router",
                    get_router_id(routers[west_out]),
                    "Router", get_router_id(routers[east_in]),
                    link_count, link_latency, iterp_width)

                    link_count += 1

        # North output to South input links (weight = 2)
        for col in range(3):
            for row in range(4):
                if (row + 1 < 4):
                    north_out = interp_router_offset + col + (row * 3)
                    south_in = interp_router_offset + col + ((row + 1) * 3)
                    int_links.append(IntLink(link_id=link_count,
                                             src_node=routers[north_out],
                                             dst_node=routers[south_in],
                                             src_serdes = False,
                                             dst_serdes = False,
                                             src_outport="North",
                                             dst_inport="South",
                                             sni=False,
                                             width = iterp_width,
                                             latency = 4,
                                             weight=2))
                    print_connection("Router",
                    get_router_id(routers[north_out]),
                    "Router", get_router_id(routers[south_in]),
                    link_count, link_latency, iterp_width)

                    link_count += 1

        # South output to North input links (weight = 2)
        for col in range(3):
            for row in range(4):
                if (row + 1 < 4):
                    north_in = interp_router_offset + col + (row * 3)
                    south_out = interp_router_offset + col + ((row + 1) * 3)
                    int_links.append(IntLink(link_id=link_count,
                                             src_node=routers[south_out],
                                             dst_node=routers[north_in],
                                             src_serdes = False,
                                             dst_serdes = False,
                                             src_outport="South",
                                             dst_inport="North",
                                             sni=False,
                                             width = iterp_width,
                                             latency = 4,
                                             weight=2))
                    print_connection("Router",
                    get_router_id(routers[north_out]),
                    "Router", get_router_id(routers[south_in]),
                    link_count, link_latency, iterp_width)

                    link_count += 1

    #=============================================================== 
    #                 MC Routers to Interposer Connection
    #===============================================================
        for (i, n) in enumerate(mc_nodes):                               
            if (i==0):
                int_links.append(IntLink(link_id=link_count,
                                         src_node=routers[73],
                                         dst_node=routers[84],
                                         src_serdes = False,
                                         dst_serdes = False,
                                         sni=False,
                                         width = iterp_width,
                                         latency = 4,
                                         weight=1))
                                                                            
                print_connection("Router", \
                get_router_id(routers[router_cores]), \
                "Router", get_router_id(routers[chiplet_xbar_id]),
                link_count, link_latency, iterp_width)
                link_count += 1
                int_links.append(IntLink(link_id=link_count,
                                         src_node=routers[84],
                                         dst_node=routers[73],
                                         src_serdes = False,
                                         dst_serdes = False,
                                         sni=False,
                                         width = iterp_width,
                                         latency = 4,
                                         weight=1))
                                                                            
                print_connection("Router", \
                get_router_id(routers[chiplet_xbar_id]), \
                "Router", get_router_id(routers[router_cores]),
                link_count, link_latency, iterp_width)
                link_count += 1
            elif (i==1):
                int_links.append(IntLink(link_id=link_count,
                                         src_node=routers[76],
                                         dst_node=routers[85],
                                         src_serdes = False,
                                         dst_serdes = False,
                                         sni=False,
                                         width = iterp_width,
                                         latency = 4,
                                         weight=1))
                                                                            
                print_connection("Router", \
                get_router_id(routers[router_cores]), \
                "Router", get_router_id(routers[chiplet_xbar_id]),
                link_count, link_latency, iterp_width)
                link_count += 1
                int_links.append(IntLink(link_id=link_count,
                                         src_node=routers[85],
                                         dst_node=routers[76],
                                         src_serdes = False,
                                         dst_serdes = False,
                                         sni=False,
                                         width = iterp_width,
                                         latency = 4,
                                         weight=1))
                                                                            
                print_connection("Router", \
                get_router_id(routers[chiplet_xbar_id]), \
                "Router", get_router_id(routers[router_cores]),
                link_count, link_latency, iterp_width)
                link_count += 1
            elif (i==2):
                int_links.append(IntLink(link_id=link_count,
                                         src_node=routers[79],
                                         dst_node=routers[86],
                                         src_serdes = False,
                                         dst_serdes = False,
                                         sni=False,
                                         width = iterp_width,
                                         latency = 4,
                                         weight=1))
                                                                            
                print_connection("Router", \
                get_router_id(routers[router_cores]), \
                "Router", get_router_id(routers[chiplet_xbar_id]),
                link_count, link_latency, iterp_width)
                link_count += 1
                int_links.append(IntLink(link_id=link_count,
                                         src_node=routers[86],
                                         dst_node=routers[79],
                                         src_serdes = False,
                                         dst_serdes = False,
                                         sni=False,
                                         width = iterp_width,
                                         latency = 4,
                                         weight=1))
                                                                            
                print_connection("Router", \
                get_router_id(routers[chiplet_xbar_id]), \
                "Router", get_router_id(routers[router_cores]),
                link_count, link_latency, iterp_width)
                link_count += 1
            elif (i==3):
                int_links.append(IntLink(link_id=link_count,
                                         src_node=routers[82],
                                         dst_node=routers[87],
                                         src_serdes = False,
                                         dst_serdes = False,
                                         sni=False,
                                         width = iterp_width,
                                         latency = 4,
                                         weight=1))
                                                                            
                print_connection("Router", \
                get_router_id(routers[router_cores]), \
                "Router", get_router_id(routers[chiplet_xbar_id]),
                link_count, link_latency, iterp_width)
                link_count += 1
                int_links.append(IntLink(link_id=link_count,
                                         src_node=routers[87],
                                         dst_node=routers[82],
                                         src_serdes = False,
                                         dst_serdes = False,
                                         sni=False,
                                         width = iterp_width,
                                         latency = 4,
                                         weight=1))
                                                                            
                print_connection("Router", \
                get_router_id(routers[chiplet_xbar_id]), \
                "Router", get_router_id(routers[router_cores]),
                link_count, link_latency, iterp_width)
                link_count += 1

        network.ext_links = ext_links 
        network.int_links = int_links


def get_router_id(node) :
    return str(node).split('.')[3].split('routers')[1]

def print_connection(src_type, src_id, dst_type, dst_id, link_id, lat, bw):
    print str(src_type) + "-" + str(src_id) + " connected to " + \
          str(dst_type) + "-" + str(dst_id) + " via Link-" + str(link_id) + \
         " with latency=" + str(lat) + " (cycles)" \
         " and bandwidth=" + str(bw) + " (bits)"
