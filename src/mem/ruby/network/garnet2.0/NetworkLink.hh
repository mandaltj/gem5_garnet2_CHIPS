/*
 * Copyright (c) 2008 Princeton University
 * Copyright (c) 2016 Georgia Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Niket Agarwal
 *          Tushar Krishna
 */


#ifndef __MEM_RUBY_NETWORK_GARNET2_0_NETWORKLINK_HH__
#define __MEM_RUBY_NETWORK_GARNET2_0_NETWORKLINK_HH__

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include "debug/RubyNetwork.hh"
#include "debug/RubySNI.hh"
#include "mem/ruby/network/garnet2.0/GarnetNetwork.hh"
#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/network/garnet2.0/CommonTypes.hh"
#include "mem/ruby/network/garnet2.0/flitBuffer.hh"
#include "params/NetworkLink.hh"
#include "sim/clocked_object.hh"
#include "sim/system.hh"

class GarnetNetwork;

class NetworkLink : public ClockedObject, public Consumer
{
  public:
    typedef NetworkLinkParams Params;
    NetworkLink(const Params *p);
    ~NetworkLink();

    void setLinkConsumer(Consumer *consumer);
    virtual void setVcsPerVnet(uint32_t consumerVcs);
    void setSourceQueue(flitBuffer *srcQueue, ClockedObject *srcClockObject);
    void setType(link_type type) { m_type = type; }
    link_type getType() { return m_type; }
    void print(std::ostream& out) const {}
    int get_id() const { return m_id; }
    flitBuffer *getBuffer() { return linkBuffer;}
    virtual void wakeup();

    unsigned int getLinkUtilization() const { return m_link_utilized; }
    const std::vector<unsigned int> & getVcLoad() const { return m_vc_load; }

    inline bool isReady(Tick curTime)
    {
        return linkBuffer->isReady(curTime);
    }

    void init_net_ptr(GarnetNetwork* net_ptr)
    {
        m_network_ptr = net_ptr;
    }

    inline flit* peekLink()       { return linkBuffer->peekTopFlit(); }
    inline flit* consumeLink()    { return linkBuffer->getTopFlit(); }

    uint32_t functionalWrite(Packet *);
    void resetStats();

    std::vector<int> mVnets;
    uint32_t bitWidth;
    bool enSNI;

    //System which manages virt->phys allocation
    System *system; 

  private:
    const int m_id;
    link_type m_type;
    const Cycles m_latency;
    //Added by TJ
    GarnetNetwork *m_network_ptr; 


  protected:
    flitBuffer *linkBuffer;
    Consumer *link_consumer;
    flitBuffer *link_srcQueue;
    ClockedObject *src_object;

    // Statistical variables
    unsigned int m_link_utilized;
    std::vector<unsigned int> m_vc_load;
    uint32_t m_virt_nets;

/*
    //Added by TJ
  private:
    struct APU{
        unsigned int apumid;
        uint64_t apuaddr_start;
        uint64_t apuaddr_end;
        unsigned int apuperm;

        APU(uint64_t id, uint64_t addr_start, uint64_t addr_end, uint64_t perm)
        : apumid(id),
          apuaddr_start(addr_start),
          apuaddr_end(addr_end),
          apuperm(perm) {}
    };

    static std::vector<APU> APU_policy_NL;

    static std::vector<APU> apu_policy_init_NL();
*/

    bool apu_check(flit *t_flit);

    //void dir_coherence_check(flit *t_flit);

    //flit* flitisizeMessage_NL(MsgPtr msg_ptr, int vnet);    

    //Takes msg_ptr and vnet as arguments and returns a 
    //pointer to a flit. flit is created based on the
    //information contained in msg_ptr and vnet 
    //flit* flitisizeMessage_NL(MsgPtr msg_ptr, int vnet);
    
    void print_APU_Table();

};

#endif // __MEM_RUBY_NETWORK_GARNET2_0_NETWORKLINK_HH__
