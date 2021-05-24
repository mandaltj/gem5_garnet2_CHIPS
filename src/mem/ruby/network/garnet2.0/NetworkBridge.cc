/*
 * Copyright (c) 2018 Advanced Micro Devices, Inc.
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
 * Authors: Srikant Bharadwaj
 */


#include "mem/ruby/network/garnet2.0/NetworkBridge.hh"

#include <cmath>
#include <fstream>
#include <sstream>
#include <vector>

#include "debug/RubyNetwork.hh"
#include "debug/RubySNI.hh"
#include "mem/ruby/slicc_interface/Message.hh"
#include "mem/ruby/slicc_interface/RubyRequest.hh"
#include "params/GarnetIntLink.hh"

NetworkBridge::NetworkBridge(const Params *p)
    :CreditLink(p)
{
    enCdc = true;
    enSerDes = true;
    mType = p->vtype;


    cdcLatency = p->cdc_latency;
    serDesLatency = p->serdes_latency;
    lastScheduledAt = 0;

    nLink = p->link;
    if (mType == FROM_LINK_) {
        nLink->setLinkConsumer(this);
        setSourceQueue(nLink->getBuffer(), nLink);
    } else if (mType == TO_LINK_) {
        nLink->setSourceQueue(linkBuffer, this);
        setLinkConsumer(nLink);
    } else {
        // CDC type must be set
        panic("CDC type must be set");
    }

}

void
NetworkBridge::setVcsPerVnet(uint32_t consumerVcs)
{
    DPRINTF(RubyNetwork, "VcsPerVnet VC: %d\n", consumerVcs);
    NetworkLink::setVcsPerVnet(consumerVcs);
    lenBuffer.resize(consumerVcs * m_virt_nets);
    sizeSent.resize(consumerVcs * m_virt_nets);
    flitsSent.resize(consumerVcs * m_virt_nets);
    extraCredit.resize(consumerVcs * m_virt_nets);

    nLink->setVcsPerVnet(consumerVcs);
}

void
NetworkBridge::init(NetworkBridge *coBrid, bool cdc_en, bool serdes_en)
{
    coBridge = coBrid;
    enCdc = cdc_en;
    enSerDes = serdes_en;
}

NetworkBridge::~NetworkBridge()
{
}

void
NetworkBridge::calculateLatency()
{
    // Change any calculations for cdc latency here
    // cdcLatency = Cycles(1);
}

void
NetworkBridge::scheduleFlit(flit *t_flit, Cycles latency)
{
    Cycles totLatency = latency;

    if (enCdc) {
        // Add the CDC latency
        totLatency = latency + cdcLatency;
    }

    Tick sendTime = link_consumer->getObject()->clockEdge(totLatency);
    Tick nextAvailTick = lastScheduledAt + link_consumer->getObject()->\
            cyclesToTicks(Cycles(1));
    sendTime = std::max(nextAvailTick, sendTime);
    t_flit->set_time(sendTime);
    lastScheduledAt = sendTime;
    linkBuffer->insert(t_flit);
    link_consumer->scheduleEventAbsolute(sendTime);
}

void
NetworkBridge::neutralize(int vc, int eCredit)
{
    extraCredit[vc].push(eCredit);
}

void
NetworkBridge::flitisizeAndSend(flit *t_flit)
{
    // Serialize-Deserialize only if it is enabled
    if (enSerDes) {
        // Calculate the target-width
        int target_width = bitWidth;
        int cur_width = nLink->bitWidth;
        if (mType == TO_LINK_) {
            target_width = nLink->bitWidth;
            cur_width = bitWidth;
        }

        DPRINTF(RubyNetwork, "Target width: %d Current: %d\n",
            target_width, cur_width);
        DPRINTF(RubySNI, "Target width: %d Current: %d\n",
            target_width, cur_width);
        assert(target_width != cur_width);

        int vc = t_flit->get_vc();

        if (target_width > cur_width) {
            // Deserialize
            // This deserializer combines flits from the
            // same message together
            int num_flits = 0;
            int flitPossible = 0;
            if (t_flit->get_type() == CREDIT_) {
                lenBuffer[vc]++;
                assert(extraCredit[vc].front());
                if (lenBuffer[vc] == extraCredit[vc].front()) {
                    flitPossible = 1;
                    extraCredit[vc].pop();
                    lenBuffer[vc] = 0;
                }
            } else if (t_flit->get_type() == TAIL_ ||
                       t_flit->get_type() == HEAD_TAIL_) {
                // If its the end of packet, then send whatever
                // is available.
                int sizeAvail = (t_flit->msgSize - sizeSent[vc]);
                flitPossible = ceil((float)sizeAvail/(float)target_width);
                assert (flitPossible < 2);
                num_flits = (t_flit->get_id() + 1) - flitsSent[vc];
                // Stop tracking the packet.
                flitsSent[vc] = 0;
                sizeSent[vc] = 0;
            } else {
                // If we are yet to receive the complete packet
                // track the size recieved and flits deserialized.
                int sizeAvail =
                    ((t_flit->get_id() + 1)*cur_width) - sizeSent[vc];
                flitPossible = floor((float)sizeAvail/(float)target_width);
                assert (flitPossible < 2);
                num_flits = (t_flit->get_id() + 1) - flitsSent[vc];
                if (flitPossible) {
                    sizeSent[vc] += target_width;
                    flitsSent[vc] = t_flit->get_id() + 1;
                }
            }

            DPRINTF(RubyNetwork, "Deserialize :%dB -----> %dB "
                " vc:%d\n", cur_width, target_width, vc);

            flit *fl = NULL;
            if (flitPossible) {
                fl = t_flit->deserialize(lenBuffer[vc], num_flits,
                    target_width);
            }

            // Inform the credit serializer about the number
            // of flits that were generated.
            if (t_flit->get_type() != CREDIT_ && fl) {
                coBridge->neutralize(vc, num_flits);
            }

            // Schedule only if we are done deserializing
            if (fl) {
                DPRINTF(RubyNetwork, "Scheduling a flit\n");
                lenBuffer[vc] = 0;
                scheduleFlit(fl, serDesLatency);
            }
            // Delete this flit, new flit is sent in any case
            delete t_flit;
        } else {
            // Serialize
            DPRINTF(RubyNetwork, "Serializing flit :%d -----> %d "
            "(vc:%d, Original Message Size: %d)\n",
                cur_width, target_width, vc, t_flit->msgSize);

            int flitPossible = 0;
            if (t_flit->get_type() == CREDIT_) {
                // We store the deserialization ratio and then
                // access it when serializing credits in the
                // oppposite direction.
                assert(extraCredit[vc].front());
                flitPossible = extraCredit[vc].front();
                extraCredit[vc].pop();
            } else if (t_flit->get_type() == HEAD_ ||
                    t_flit->get_type() == BODY_) {
                int sizeAvail =
                    ((t_flit->get_id() + 1)*cur_width) - sizeSent[vc];
                flitPossible = floor((float)sizeAvail/(float)target_width);
                if (flitPossible) {
                    sizeSent[vc] += flitPossible*target_width;
                    flitsSent[vc] += flitPossible;
                }
            } else {
                int sizeAvail = t_flit->msgSize - sizeSent[vc];
                flitPossible = ceil((float)sizeAvail/(float)target_width);
                sizeSent[vc] = 0;
                flitsSent[vc] = 0;
            }
            assert(flitPossible > 0);

            // Schedule all the flits
            // num_flits could be zero for credits
            for (int i = 0; i < flitPossible; i++) {
                // Ignore neutralized credits
                flit *fl = t_flit->serialize(i, flitPossible, target_width);
                scheduleFlit(fl, serDesLatency);
                DPRINTF(RubyNetwork, "Serialized to flit[%d of %d parts]:"
                " %s\n", i+1, flitPossible, *fl);
            }

            if (t_flit->get_type() != CREDIT_) {
                coBridge->neutralize(vc, flitPossible);
            }
            // Delete this flit, new flit is sent in any case
            delete t_flit;
        }
        return;
    }

    // If only CDC is enabled schedule it
    scheduleFlit(t_flit, Cycles(0));
}
void
NetworkBridge::wakeup()
{
    // Update CDC latency
    calculateLatency();

    flit *t_flit;

    if (link_srcQueue->isReady(curTick())) {
        t_flit = link_srcQueue->getTopFlit();

        DPRINTF(RubySNI, "SNI: Network Bridge Wakeup\n");

        //TJ: For Flits of type Credit, t_flit->get_msg_ptr()
        //is nullptr and hence will give segmentation fault
        //on dereference
        if (t_flit->get_type() != CREDIT_){
            Addr addr_check = (t_flit->get_msg_ptr())->getaddr();
            DPRINTF(RubyNetwork, "Line_Address: 0x%x\n",
            addr_check);

            DPRINTF(RubyNetwork, "SNI: Network Bridge wakeup: \
            Recieved flit %s; Message: %s\n",
            *t_flit, *(t_flit->get_msg_ptr()));
        }

        flitisizeAndSend(t_flit);
        return;
    }
    assert(!link_srcQueue->getSize());
}

NetworkBridge *
NetworkBridgeParams::create()
{
    return new NetworkBridge(this);
}

/*
//Added by TJ
std::vector<NetworkBridge::APU> NetworkBridge::apu_policy_init(){
    std::vector<APU> APU_temp;

    std::string filename = "my_benchmarks/apu_init/apu_init.txt";
    std::ifstream infile(filename);
    if (!infile) {
        infile.close();
        //std::cout<<"Current Directory: "<<get_current_dir()<<'\n';
        assert(false);
    }

    uint64_t id;
    uint64_t start_addr;
    uint64_t end_addr;
    uint64_t perm;

    while (infile >> id >> start_addr >> end_addr >> perm){
        APU_temp.emplace_back(id, start_addr, end_addr, perm);
    }

    return APU_temp;
}

bool NetworkBridge::apu_check(flit *t_flit){
    Addr addr_check = (t_flit->get_msg_ptr())->getaddr();
    //THe chiplet_id is being extracted from the
    //L2Cahce-0/1/2/3... NodeID number Check getRequestor()
    //in RequestMsg/ResponseMsg and then the MachineID structure
    //Since each chiplet has a single L2 Cache,
    //the node number of L2 cache will be equal to the Chiplet ID
    unsigned int requestor_id = (unsigned int)((t_flit->get_msg_ptr())
                              ->getRequestor()).getNum();
    unsigned int chiplet_id = requestor_id%8;
    DPRINTF(RubySNI, "TJ: Requestor:%d, Chiplet_ID:%d,
    Addr:%d\n", requestor_id, chiplet_id, addr_check);

    //unsigned int request_type = (t_flit->get_msg_ptr())->getreqtype();
    int apu_policy_size = APU_policy.size();
    for (int i=0; i<apu_policy_size; i++){
        //Read type - GETX and GETS
        //Write type -
            if ((addr_check>=APU_policy[i].apuaddr_start)
               && (addr_check<=APU_policy[i].apuaddr_end)){
               switch(chiplet_id) {
                  case 0:
                      if (APU_policy[i].apumid && 0x1 == 0){
                           assert(false);
                           return false;
                      }else{
                           return true;
                      }
                  case 1:
                      if (APU_policy[i].apumid && 0x2 == 0){
                           assert(false);
                           return false;
                      }else{
                           return true;
                      }
                  case 2:
                      if (APU_policy[i].apumid && 0x4 == 0){
                          assert(false);
                          return false;
                      }else{
                          return true;
                      }
                  case 3:
                      if (APU_policy[i].apumid && 0x8 == 0){
                          assert(false);
                          return false;
                      }else{
                          return true;
                      }
                  case 4:
                      if (APU_policy[i].apumid && 0x10 == 0){
                          assert(false);
                          return false;
                      }else{
                          return true;
                      }
                  case 5:
                      if (APU_policy[i].apumid && 0x20 == 0){
                           assert(false);
                           return false;
                      }else{
                           return true;
                      }
                  case 6:
                      if (APU_policy[i].apumid && 0x40 == 0){
                           assert(false);
                           return false;
                      }else{
                           return true;
                      }
                  case 7:
                      if (APU_policy[i].apumid && 0x80 == 0){
                          assert(false);
                          return false;
                      }else{
                          return true;
                      }
               }
                //Pass-Do Nothing!!
                //Add delay/latency for this check
            }
            else{
                //Fails
                //So we want to generate a response;
                //Who creates the dummy response packet?
                DPRINTF(RubySNI, "SNI: What went wrong?\n");
                assert(false);
                return false;
            }
    }
    //Don't think we should ever end up here
    return false;
}
*/

