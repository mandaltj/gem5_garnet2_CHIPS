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

class CHIPS_GTRocket_XBAR(SimpleTopology):
    description = 'CHIPS_GTRocket_XBAR'

    def __init__(self, controllers):
        self.nodes = controllers

    def makeTopology(self, options, network, IntLink, ExtLink, Router):
        nodes = self.nodes

        # default values for link latency and router latency.
        # Can be over-ridden on a per link/router basis
        link_latency = options.link_latency # used by simple and garnet
        router_latency = options.router_latency # only used by garnet

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

        #Assuming 8 Chiplets
        num_cpu_chiplets = options.num_chiplets
        #Assuming 64 Cores; 8 Chiplets each containing 8 Cores
        num_cpus_per_chiplet = len(cpu_nodes)/num_cpu_chiplets
        #MemoryController Chiplets; Assuming 4
        num_mc_chiplets = options.num_dirs

        #Mesh rows and columns
        #CPUs in a chiplet are being connected using a Full xBar
        #Hence, skipping the rows/columns calculation

        num_routers = len(cpu_nodes) + num_mc_chiplets + num_cpu_chiplets + 1

        # Create the routers in the mesh
        routers = [Router(router_id=i, latency = router_latency) \
            for i in range(num_routers)]
        network.routers = routers

        # link counter to set unique link ids
        link_count = 0

        # start from router 0
        router_id = 0

        print("=====================================")
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

        # Connect each CPU to a unique router
        # Router 0 to 63 connect to Core 0 to 63
        for (i, n) in enumerate(cpu_nodes):
            cpu_link_width = 32
            routers[router_id].width = cpu_link_width
            ext_links.append(ExtLink(link_id=link_count, ext_node=n,
                                    int_node=routers[router_id],
                                    width=cpu_link_width,
                                    int_serdes = False,
                                    latency = link_latency))
            print_connection("CPU", n.version, "Router", router_id, \
            link_count, link_latency, cpu_link_width)
            link_count += 1
            router_id += 1

        #assert(router_id==64)

        # Connect the MC nodes to routers
        # Router 64 to 67 connect to 4 MCs
        for (i, n) in enumerate(mc_nodes):
            mc_link_width = 32
            routers[router_id].width = mc_link_width
            ext_links.append(ExtLink(link_id=link_count, ext_node=n,
                                    int_node=routers[router_id],
                                    width=mc_link_width,
                                    int_serdes = False,
                                    latency = link_latency))
            print_connection("MC", n.version, "Router", router_id,
            link_count, link_latency, mc_link_width)
            link_count += 1
            router_id += 1

        #assert(router_id==68)

        for (i, n) in enumerate(l2_nodes):
            l2_link_width = 32
            routers[router_id].width = l2_link_width
            ext_links.append(ExtLink(link_id=link_count, ext_node=n,
                                    int_node=routers[router_id],
                                    width=l2_link_width,
                                    int_serdes = False,
                                    latency = link_latency))
            print_connection("L2", n.version, "Router", router_id, \
            link_count, link_latency, l2_link_width)
            link_count += 1
            router_id += 1

        #assert(router_id==76)

        network.ext_links = ext_links

        #All routers except xbar in each chiplet
        #and the main xBar have been connected

        #============================================================
        #Connect the Internal Links between Routers in each Chiplet
        #SerDes = False
        #============================================================
        # Create the mesh links inside each chiplet
        int_links = []

        #There are 8 Cores in each Chiplet. Each Core is connected
        #to a Full System Xbar. Router 0-7 will act as sources and Router
        chiplet_xbar_id = len(cpu_nodes) + num_mc_chiplets
        #Idea is the the full systemxbar will have the ids
        #68,69,70,71,72,73,74,75
        for cc in xrange(num_cpu_chiplets):
            #assert(chiplet_xbar_id>=68 and chiplet_xbar_id<=75)
            #We can assign a high latency for the xbar router
            routers[chiplet_xbar_id].latency = 4
            for cpu in xrange(num_cpus_per_chiplet):
                chiplet_intlink_width = 32
                router_cores = (cc*num_cpus_per_chiplet)+cpu
                int_links.append(IntLink(link_id=link_count,
                                         src_node=routers[router_cores],
                                         dst_node=routers[chiplet_xbar_id],
                                         src_serdes = False,
                                         dst_serdes = False,
                                         width = chiplet_intlink_width,
                                         latency = link_latency,
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
                                         width = chiplet_intlink_width,
                                         latency = link_latency,
                                         weight=1))

                print_connection("Router", \
                get_router_id(routers[chiplet_xbar_id]), \
                "Router", get_router_id(routers[router_cores]),
                link_count, link_latency, chiplet_intlink_width)


                link_count += 1
            router_id += 1
            chiplet_xbar_id += 1

        #===============================================================
        #                 SerDes Connections Interposer
        #===============================================================

        noc_xbar_id = num_routers-1
        #assert(noc_xbar_id==76)
        #Reset the chiplet Xbar ids
        chiplet_xbar_id = len(cpu_nodes) + num_mc_chiplets
        #Now we connect the full xbar of each chiplet to the NOC
        #Increasing the latency of the NOC xbar
        #routers[noc_xbar_id].latency = 4
        iterp_width = 32

        for cc in xrange(num_cpu_chiplets):
            int_links.append(IntLink(link_id=link_count,
                                     src_node=routers[noc_xbar_id],
                                     dst_node=routers[chiplet_xbar_id],
                                     src_serdes = False,
                                     dst_serdes = False,
                                     width = iterp_width,
                                     latency = link_latency,
                                     weight=1))
            print_connection("Router", get_router_id(routers[noc_xbar_id]),
            "Router", get_router_id(routers[chiplet_xbar_id]),
            link_count, link_latency, iterp_width)

            link_count += 1

            int_links.append(IntLink(link_id=link_count,
                                     src_node=routers[chiplet_xbar_id],
                                     dst_node=routers[noc_xbar_id],
                                     src_serdes = False,
                                     dst_serdes = False,
                                     width = iterp_width,
                                     latency = link_latency,
                                     weight=1))

            print_connection("Router", get_router_id(routers[chiplet_xbar_id]),
            "Router", get_router_id(routers[noc_xbar_id]),
            link_count, link_latency, iterp_width)

            link_count += 1
            chiplet_xbar_id += 1


        #Connecting MC chiplets to NOC xBar through interposer
        mc_router_id = len(cpu_nodes)
        for mc in xrange(num_mc_chiplets):
            int_links.append(IntLink(link_id=link_count,
                                     src_node=routers[mc_router_id],
                                     dst_node=routers[noc_xbar_id],
                                     src_serdes = False,
                                     dst_serdes = False,
                                     width = iterp_width,
                                     latency = link_latency,
                                     weight=1))
            print_connection("Router", get_router_id(routers[mc_router_id]),
            "Router", get_router_id(routers[noc_xbar_id]),
            link_count, link_latency, iterp_width)
            link_count += 1

            int_links.append(IntLink(link_id=link_count,
                                     src_node=routers[noc_xbar_id],
                                     dst_node=routers[mc_router_id],
                                     src_serdes = False,
                                     dst_serdes = False,
                                     width = iterp_width,
                                     latency = link_latency,
                                     weight=1))
            print_connection("Router", get_router_id(routers[noc_xbar_id]),
            "Router", get_router_id(routers[mc_router_id]),
            link_count, link_latency, iterp_width)
            link_count += 1
            mc_router_id += 1;


        network.int_links = int_links

def get_router_id(node) :
    return str(node).split('.')[3].split('routers')[1]

def print_connection(src_type, src_id, dst_type, dst_id, link_id, lat, bw):
    print str(src_type) + "-" + str(src_id) + " connected to " + \
          str(dst_type) + "-" + str(dst_id) + " via Link-" + str(link_id) + \
         " with latency=" + str(lat) + " (cycles)" \
         " and bandwidth=" + str(bw) + " (bits)"
