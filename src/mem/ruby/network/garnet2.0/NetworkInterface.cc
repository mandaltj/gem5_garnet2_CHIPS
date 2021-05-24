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


#include "mem/ruby/network/garnet2.0/NetworkInterface.hh"

#include <cassert>
#include <cmath>

#include "base/cast.hh"
#include "base/stl_helpers.hh"
#include "debug/RubyNetwork.hh"
#include "mem/ruby/network/MessageBuffer.hh"
#include "mem/ruby/network/garnet2.0/Credit.hh"
#include "mem/ruby/network/garnet2.0/flitBuffer.hh"
#include "mem/ruby/slicc_interface/Message.hh"
#include "mem/ruby/protocol/ResponseMsg.hh"

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h> 

using namespace std;
using m5::stl_helpers::deletePointers;

//std::vector<NetworkInterface::APU> NetworkInterface::APU_policy_NI = apu_policy_init_NI();

NetworkInterface::NetworkInterface(const Params *p)
    : ClockedObject(p), Consumer(this), 
      system(p->system),  
      m_id(p->id),
      m_virtual_networks(p->virt_nets),
      m_deadlock_threshold(p->garnet_deadlock_threshold),
      vc_busy_counter(m_virtual_networks, 0)
{
    m_num_vcs = 0;
    m_vc_per_vnet = 0;
    m_vc_allocator.resize(m_virtual_networks); // 1 allocator per vnet
    for (int i = 0; i < m_virtual_networks; i++) {
        m_vc_allocator[i] = 0;
    }
    if(p->id == 64 || p->id == 65 || p->id == 66 || p->id == 67){
        enSNI=true;
        //enSNI=false;
        //if(p->sni == 1){
        //    enSNI = true;
        //    DPRINTF(RubySNI, "Net_Intf: SNI Enabled at NI\n");      
        //}else{
        //    enSNI = false;
        //    DPRINTF(RubySNI, "Net_Intf: SNI Disabled at NI\n");       
        //}
        //DPRINTF(RubySNI, "Net_Intf: Error\n");       
    } else {
        enSNI = false;
    }  
    m_stall_count.resize(m_virtual_networks);
}

void
NetworkInterface::init()
{
    return;
}

NetworkInterface::~NetworkInterface()
{
    deletePointers(m_out_vc_state);
    deletePointers(m_ni_out_vcs);
}

void
NetworkInterface::addInPort(NetworkLink *in_link,
                              CreditLink *credit_link)
{
    InputPort *newInPort = new InputPort(in_link, credit_link);
    inPorts.push_back(newInPort);
    DPRINTF(RubyNetwork, "Adding input port:%s with vnets %s\n",
    in_link->name(), newInPort->printVnets());

    in_link->setLinkConsumer(this);
    credit_link->setSourceQueue(newInPort->outCreditQueue(), this);
    if (m_vc_per_vnet != 0) {
        in_link->setVcsPerVnet(m_vc_per_vnet);
        credit_link->setVcsPerVnet(m_vc_per_vnet);
    }

}

void
NetworkInterface::addOutPort(NetworkLink *out_link,
                             CreditLink *credit_link,
                             SwitchID router_id, uint32_t consumerVcs)
{
    OutputPort *newOutPort = new OutputPort(out_link, credit_link, router_id);
    outPorts.push_back(newOutPort);

    assert(consumerVcs > 0);
    // We are not allowing different physical links to have different vcs
    // If it is required that the Network Interface support different VCs
    // for every physical link connected to it. Then they need to change
    // the logic within outport and inport.
    if (m_num_vcs == 0) {
        m_vc_per_vnet = consumerVcs;
        m_num_vcs = consumerVcs * m_virtual_networks;
        m_ni_out_vcs.resize(m_num_vcs);
        m_ni_out_vcs_enqueue_time.resize(m_num_vcs);
        // instantiating the NI flit buffers
        for (int i = 0; i < m_num_vcs; i++) {
            m_ni_out_vcs[i] = new flitBuffer();
            m_ni_out_vcs_enqueue_time[i] = Tick(INFINITE_);
            m_out_vc_state.push_back(new OutVcState(i, m_net_ptr,
                consumerVcs));
        }

        // Reset VC Per VNET for input links already instantiated
        for (auto &iPort: inPorts) {
            NetworkLink *inNetLink = iPort->inNetLink();
            inNetLink->setVcsPerVnet(m_vc_per_vnet);
            credit_link->setVcsPerVnet(m_vc_per_vnet);
        }
    } else {
        fatal_if(consumerVcs != m_vc_per_vnet,
        "%s: Connected Physical links have different vc requests: %d and %d\n",
        name(), consumerVcs, m_vc_per_vnet);
    }

    DPRINTF(RubyNetwork, "OutputPort:%s Vnet: %s\n",
    out_link->name(), newOutPort->printVnets());

    out_link->setSourceQueue(newOutPort->outFlitQueue(), this);
    out_link->setVcsPerVnet(m_vc_per_vnet);
    credit_link->setLinkConsumer(this);
    credit_link->setVcsPerVnet(m_vc_per_vnet);
}

void
NetworkInterface::addNode(vector<MessageBuffer *>& in,
                            vector<MessageBuffer *>& out)
{
    inNode_ptr = in;
    outNode_ptr = out;

    for (auto& it : in) {
        if (it != nullptr) {
            it->setConsumer(this);
        }
    }
}

void
NetworkInterface::dequeueCallback()
{
    // An output MessageBuffer has dequeued something this cycle and there
    // is now space to enqueue a stalled message. However, we cannot wake
    // on the same cycle as the dequeue. Schedule a wake at the soonest
    // possible time (next cycle).
    scheduleEventAbsolute(clockEdge(Cycles(1)));
}

void
NetworkInterface::incrementStats(flit *t_flit)
{
    int vnet = t_flit->get_vnet();

    // Latency
    m_net_ptr->increment_received_flits(vnet);
    Tick network_delay =
        t_flit->get_dequeue_time() -
        t_flit->get_enqueue_time() - cyclesToTicks(Cycles(1));
    Tick src_queueing_delay = t_flit->get_src_delay();
    Tick dest_queueing_delay = (curTick() - t_flit->get_dequeue_time());
    Tick queueing_delay = src_queueing_delay + dest_queueing_delay;

    m_net_ptr->increment_flit_network_latency(network_delay, vnet);
    m_net_ptr->increment_flit_queueing_latency(queueing_delay, vnet);

    if (t_flit->get_type() == TAIL_ || t_flit->get_type() == HEAD_TAIL_) {
        m_net_ptr->increment_received_packets(vnet);
        m_net_ptr->increment_packet_network_latency(network_delay, vnet);
        m_net_ptr->increment_packet_queueing_latency(queueing_delay, vnet);
    }

    // Hops
    m_net_ptr->increment_total_hops(t_flit->get_route().hops_traversed);
}

/*
 * The NI wakeup checks whether there are any ready messages in the protocol
 * buffer. If yes, it picks that up, flitisizes it into a number of flits and
 * puts it into an output buffer and schedules the output link. On a wakeup
 * it also checks whether there are flits in the input link. If yes, it picks
 * them up and if the flit is a tail, the NI inserts the corresponding message
 * into the protocol buffer. It also checks for credits being sent by the
 * downstream router.
 */

void
NetworkInterface::wakeup()
{
    std::ostringstream oss;
    for (auto &oPort: outPorts) {
        oss << oPort->routerID() << "[" << oPort->printVnets() << "] ";
    }
    DPRINTF(RubyNetwork, "Network Interface %d connected to router:%s "
            "woke up. Period: %ld\n", m_id, oss.str(), clockPeriod());

    assert(curTick() == clockEdge());
    MsgPtr msg_ptr;
    Tick curTime = clockEdge();


    DPRINTF(RubySNI, "Network Interface: Wakeup time:%ld\n",clockEdge(Cycles(1)));

    // Checking for messages coming from the protocol
    // can pick up a message/cycle for each virtual net
    for (int vnet = 0; vnet < inNode_ptr.size(); ++vnet) {
        MessageBuffer *b = inNode_ptr[vnet];
        if (b == nullptr) {
            continue;
        }

        if (b->isReady(curTime)) { // Is there a message waiting
            msg_ptr = b->peekMsgPtr();
            if (flitisizeMessage(msg_ptr, vnet)) {
                b->dequeue(curTime);
            }
        }
    }

    scheduleOutputLink();

    // Check if there are flits stalling a virtual channel. Track if a
    // message is enqueued to restrict ejection to one message per cycle.
    checkStallQueue();

    /*********** Check the incoming flit link **********/
    DPRINTF(RubyNetwork, "Number of input ports: %d\n", inPorts.size());
    for (auto &iPort: inPorts) {
        NetworkLink *inNetLink = iPort->inNetLink();
        if (inNetLink->isReady(curTick())) {
            flit *t_flit = inNetLink->consumeLink();
            DPRINTF(RubyNetwork, "Recieved flit:%s\n", *t_flit);
            assert(t_flit->m_width == iPort->bitWidth());

            int vnet = t_flit->get_vnet();
            t_flit->set_dequeue_time(curTick());

            // If a tail flit is received, enqueue into the protocol buffers
            // if space is available. Otherwise, exchange non-tail flits for
            // credits.
            if (t_flit->get_type() == TAIL_ ||
                t_flit->get_type() == HEAD_TAIL_) {
                if (!iPort->messageEnqueuedThisCycle &&
                    outNode_ptr[vnet]->areNSlotsAvailable(1, curTime)) {
                    // Space is available. Enqueue to protocol buffer.
                    outNode_ptr[vnet]->enqueue(t_flit->get_msg_ptr(), curTime,
                                               cyclesToTicks(Cycles(1)));

                    // Simply send a credit back since we are not buffering
                    // this flit in the NI
                    Credit *cFlit = new Credit(t_flit->get_vc(),
                                               true, curTick());
                    iPort->sendCredit(cFlit);
                    // Update stats and delete flit pointer
                    incrementStats(t_flit);
                    delete t_flit;
                } else {
                    // No space available- Place tail flit in stall queue and
                    // set up a callback for when protocol buffer is dequeued.
                    // Stat update and flit pointer deletion will occur upon
                    // unstall.
                    iPort->m_stall_queue.push_back(t_flit);
                    m_stall_count[vnet]++;

                    auto cb = std::bind(&NetworkInterface::dequeueCallback,
                                           this);
                    outNode_ptr[vnet]->registerDequeueCallback(cb);
                }
            } else {
                // Non-tail flit. Send back a credit but not VC free signal.
                Credit *cFlit = new Credit(t_flit->get_vc(), false,
                                               curTick());
                // Simply send a credit back since we are not buffering
                // this flit in the NI
                iPort->sendCredit(cFlit);

                // Update stats and delete flit pointer.
                incrementStats(t_flit);
                delete t_flit;
            }
        }
    }

    /****************** Check the incoming credit link *******/

    for (auto &oPort: outPorts) {
        CreditLink *inCreditLink = oPort->inCreditLink();
        if (inCreditLink->isReady(curTick())) {
            Credit *t_credit = (Credit*) inCreditLink->consumeLink();
            m_out_vc_state[t_credit->get_vc()]->increment_credit();
            DPRINTF(RubySNI, "Network Interface: Received credit for VC:%d\n",t_credit->get_vc()); 
            if (t_credit->is_free_signal()) {
                m_out_vc_state[t_credit->get_vc()]->setState(IDLE_,
                    curTick());
                DPRINTF(RubySNI, "Network Interface: Tran to Idle State for VC:%d\n",t_credit->get_vc());  
            }
            delete t_credit;
        }
    }


    // It is possible to enqueue multiple outgoing credit flits if a message
    // was unstalled in the same cycle as a new message arrives. In this
    // case, we should schedule another wakeup to ensure the credit is sent
    // back.
    for (auto &iPort: inPorts) {
        if (iPort->outCreditQueue()->getSize() > 0) {
            DPRINTF(RubyNetwork, "Sending a credit %s via %s at %ld\n",
            *(iPort->outCreditQueue()->peekTopFlit()),
            iPort->outCreditLink()->name(), clockEdge(Cycles(1)));
            iPort->outCreditLink()->
                scheduleEventAbsolute(clockEdge(Cycles(1)));
        }
    }

    checkReschedule();
}

void
NetworkInterface::checkStallQueue()
{
    // Check all stall queues.
    // There is one stall queue for each input link
    for (auto &iPort: inPorts) {
        iPort->messageEnqueuedThisCycle = false;
        Tick curTime = clockEdge();

        if (!iPort->m_stall_queue.empty()) {
            for (auto stallIter = iPort->m_stall_queue.begin();
                 stallIter != iPort->m_stall_queue.end(); ) {
                flit *stallFlit = *stallIter;
                int vnet = stallFlit->get_vnet();

                // If we can now eject to the protocol buffer,
                // send back credits
                if (outNode_ptr[vnet]->areNSlotsAvailable(1,
                    curTime)) {
                    outNode_ptr[vnet]->enqueue(stallFlit->get_msg_ptr(),
                        curTime, cyclesToTicks(Cycles(1)));

                    // Send back a credit with free signal now that the
                    // VC is no longer stalled.
                    Credit *cFlit = new Credit(stallFlit->get_vc(), true,
                                                   curTick());
                    iPort->sendCredit(cFlit);

                    // Update Stats
                    incrementStats(stallFlit);

                    // Flit can now safely be deleted and removed from stall
                    // queue
                    delete stallFlit;
                    iPort->m_stall_queue.erase(stallIter);
                    m_stall_count[vnet]--;

                    // If there are no more stalled messages for this vnet, the
                    // callback on it's MessageBuffer is not needed.
                    if (m_stall_count[vnet] == 0)
                        outNode_ptr[vnet]->unregisterDequeueCallback();

                    iPort->messageEnqueuedThisCycle = true;
                    break;
                } else {
                    ++stallIter;
                }
            }
        }
    }
}

// Embed the protocol message into flits
bool
NetworkInterface::flitisizeMessage(MsgPtr msg_ptr, int vnet)
{

    int sni_packet_delay = 0;
    if(enSNI){
        DPRINTF(RubySNI, "Network Interface: ********************************************************\n");       
        DPRINTF(RubySNI, "Network Interface: Net_Intf: id:%0d; Message obtained:%s\n", m_id, *msg_ptr);      
    } 
    Message *net_msg_ptr = msg_ptr.get();
    NetDest net_msg_dest = net_msg_ptr->getDestination();

    // gets all the destinations associated with this message.
    vector<NodeID> dest_nodes = net_msg_dest.getAllDest();

    // Number of flits is dependent on the link bandwidth available.
    // This is expressed in terms of bytes/cycle or the flit size
    
    //TJ: Make sure all the widths are 32-bit; Don't change the widths
    OutputPort *oPort = getOutportForVnet(vnet);

    assert(oPort);
    int num_flits = (int) ceil((double) m_net_ptr->MessageSizeType_to_int(
        net_msg_ptr->getMessageSize())/oPort->bitWidth());
    
    if(msg_ptr->getReqRespType() == 0){
        if( msg_ptr->getCohReqType() == 0 ||                                      
            msg_ptr->getCohReqType() == 1){
            //DPRINTF(RubySNI, "Net_Intf: id:%0d; Message obtained:%s\n", m_id, *msg_ptr);      
            if(num_flits>1){
                assert(false);
                //Since GETS and GETX are Control Packets
                //They should be of flit size 1
            } 
        }    
    }
    
    DPRINTF(RubyNetwork, "Message Size:%d vnet:%d bitWidth:%d\n",
    m_net_ptr->MessageSizeType_to_int(net_msg_ptr->getMessageSize()),
    vnet, oPort->bitWidth());

    // loop to convert all multicast messages into unicast messages
    for (int ctr = 0; ctr < dest_nodes.size(); ctr++) {
        //DPRINTF(RubySNI, "Net_Intf SNI: Iteration:%0d\n", ctr);     
        MsgPtr new_msg_ptr = msg_ptr->clone();
        NodeID destID = dest_nodes[ctr];
        DPRINTF(RubySNI, "Network Interface: Net_Intf: Iteration:%0d; destID;%0d\n", ctr, destID);       
        
        //Default Status
        bool apu_result = true; 
        if(enSNI && msg_ptr->getReqRespType() == 0){
            //TODO: Only GETS added getCohReqType==0
            //Add GETX and INV as well 
            if(new_msg_ptr->getCohReqType()==0 ||
               new_msg_ptr->getCohReqType()==1){
                DPRINTF(RubySNI, "Network Interface: NI_SNI: APU_CHECK Entered\n");    
                apu_result = apu_check_NI(new_msg_ptr, destID); 
                if(!apu_result){
                    DPRINTF(RubySNI, "Network Interface: Net_Intf SNI: Apu_Result False; Create Packet to reroute to Core\n");    
                    sni_packet_delay = 3; 
                } else {
                    DPRINTF(RubySNI, "Network Interface: net_Intf SNI: Apu_Result True; Let Packet pass as it is\n");   
                    sni_packet_delay = 1;   
                }
                DPRINTF(RubySNI, "Network Interface: NI_SNI: APU_CHECK Done\n");    
            }
        }
        
        int vc = -1; 
        if(apu_result){
            // this will return a free output virtual channel  
            vc = calculateVC(vnet); 
        }    
        else{
            // this will return a free output virtual channel 
            //This should be Virtual Network 4
            //From Cache.sm
            //MessageBuffer * responseToCache, network="From", virtual_network="4",
            //vnet_type="response"; 
            vc = calculateVC(4);
            vnet = 4;
            //Update the message and destination
            new_msg_ptr = modify_message_sni(new_msg_ptr, destID); 
            oPort = getOutportForVnet(4);
        }
         
        if(!apu_result){                                                                            
            DPRINTF(RubySNI, "Network Interface: SNI Net_Intf: ctr:%0d; new_msg_ptr:%s; \n", ctr, *new_msg_ptr);      
        }

        if (vc == -1) {
            DPRINTF(RubySNI, "Network Interface: No Virtual Channel free; Breaking out of flitisizemessage()\n");           
            return false ;
        }

        //Execute this section of code if only apu_result=true
        //In that case we are letting the message pass
        Message *new_net_msg_ptr = new_msg_ptr.get(); 
        if (dest_nodes.size() > 1) {
            NetDest personal_dest;
            for (int m = 0; m < (int) MachineType_NUM; m++) {
                if ((destID >= MachineType_base_number((MachineType) m)) &&
                    destID < MachineType_base_number((MachineType) (m+1))) {
                    // calculating the NetDest associated with this destID
                    personal_dest.clear();
                    personal_dest.add((MachineID) {(MachineType) m, (destID -
                        MachineType_base_number((MachineType) m))});
                    
                    //Added by TJ
                    if(apu_result){
                        new_net_msg_ptr->getDestination() = personal_dest; 
                    } else {
                        new_net_msg_ptr->getDestination() = new_msg_ptr->getDestination();  
                    }
                    break;
                }
            }
            net_msg_dest.removeNetDest(personal_dest);
            // removing the destination from the original message to reflect
            // that a message with this particular destination has been
            // flitisized and an output vc is acquired
            net_msg_ptr->getDestination().removeNetDest(personal_dest);
            DPRINTF(RubySNI, "Network Interface: SNI Net_Intf: personal_dest:%s;\n", personal_dest);  
        }

        //DPRINTF(RubySNI, "NI_SNI: Net Destination Updated\n");    
        //DPRINTF(RubySNI, "SNI Net_Intf: new_net_msg_ptr destination:%s;\n", new_net_msg_ptr->getDestination()); 
        //DPRINTF(RubySNI, "SNI Net_Intf: net_msg_dest destination:%s;\n", net_msg_dest); 
        //DPRINTF(RubySNI, "SNI Net_Intf: net_msg_ptr destination:%s;\n", net_msg_ptr->getDestination());          

        if(!apu_result){
            DPRINTF(RubySNI, "Network Interface: SNI Net_Intf: ctr:%0d; new_msg_ptr:%s; \n", ctr, *new_msg_ptr);      
        }
        // Embed Route into the flits
        // NetDest format is used by the routing table
        // Custom routing algorithms just need destID

        RouteInfo route;
        
        //Added by TJ
        //If "false" then we are changing the coherent message
        route.vnet = vnet;   
        if (apu_result){
            route.dest_ni = destID;
            route.dest_router = m_net_ptr->get_router_id(destID, vnet);  
        } else {
            route.dest_ni = ((new_msg_ptr->getDestination()).smallestElement()).getNum();  
            route.dest_router = m_net_ptr->get_router_id(((new_msg_ptr->getDestination()).smallestElement()).getNum(), vnet); 
        }
        route.net_dest = new_net_msg_ptr->getDestination();  
        route.src_ni = m_id;
        route.src_router = oPort->routerID();

        // initialize hops_traversed to -1
        // so that the first router increments it to 0
        route.hops_traversed = -1;
        DPRINTF(RubySNI, "Network Interface: NI_SNI: Route Info Done\n");   
         
        /*
        m_net_ptr->increment_injected_packets(vnet);
        for (int i = 0; i < num_flits; i++) {
            m_net_ptr->increment_injected_flits(vnet);
            flit *fl = new flit(i, vc, vnet, route, num_flits, new_msg_ptr,
                m_net_ptr->MessageSizeType_to_int(
                net_msg_ptr->getMessageSize()),
                oPort->bitWidth(), curTick());

            fl->set_src_delay(curTick() - (msg_ptr->getTime()));
            m_ni_out_vcs[vc]->insert(fl);
        }

        m_ni_out_vcs_enqueue_time[vc] = curTick();
        m_out_vc_state[vc]->setState(ACTIVE_, curTick());
        */
        
        m_net_ptr->increment_injected_packets(vnet);                        
        for (int i = 0; i < num_flits; i++) {
            m_net_ptr->increment_injected_flits(vnet);
            flit *fl = new flit(i, vc, vnet, route, num_flits, new_msg_ptr,
                m_net_ptr->MessageSizeType_to_int(
                net_msg_ptr->getMessageSize()),
                oPort->bitWidth(), curTick()+sni_packet_delay);
                                                                            
            fl->set_src_delay((curTick()+sni_packet_delay) - (msg_ptr->getTime()));
            m_ni_out_vcs[vc]->insert(fl);
        }
                                                                            
        m_ni_out_vcs_enqueue_time[vc] = curTick();
        m_out_vc_state[vc]->setState(ACTIVE_, curTick());
        
        
        
        
        
        if(enSNI){
            DPRINTF(RubySNI, "Network Interface: NI_SNI: flitisize message done; VC:%0d\n", vc);  
            DPRINTF(RubySNI, "Network Interface: ********************************************************\n");         
        }
         
    }
    DPRINTF(RubySNI, "Network Interface: **********Breaking Out of Network Interface**********************\n");          
    return true ;
}

// Looking for a free output vc
int
NetworkInterface::calculateVC(int vnet)
{
    for (int i = 0; i < m_vc_per_vnet; i++) {
        int delta = m_vc_allocator[vnet];
        m_vc_allocator[vnet]++;
        if (m_vc_allocator[vnet] == m_vc_per_vnet)
            m_vc_allocator[vnet] = 0;

        if (m_out_vc_state[(vnet*m_vc_per_vnet) + delta]->isInState(
                    IDLE_, curTick())) {
            vc_busy_counter[vnet] = 0;
            return ((vnet*m_vc_per_vnet) + delta);
        }
    }

    vc_busy_counter[vnet] += 1;
    panic_if(vc_busy_counter[vnet] > m_deadlock_threshold,
        "%s: Possible network deadlock in vnet: %d at time: %llu \n",
        name(), vnet, curTick());

    return -1;
}


/** This function looks at the NI buffers
 *  if some buffer has flits which are ready to traverse the link in the next
 *  cycle, and the downstream output vc associated with this flit has buffers
 *  left, the link is scheduled for the next cycle
 */

void
NetworkInterface::scheduleOutputLink()
{
    // Schedule each output link
    for (auto &oPort: outPorts) {
        int vc = oPort->vcRoundRobin();

        for (int i = 0; i < m_num_vcs; i++) {
            vc++;
            if (vc == m_num_vcs)
                vc = 0;

            int t_vnet = get_vnet(vc);
            if (oPort->isVnetSupported(t_vnet)) {
                // model buffer backpressure
                if (m_ni_out_vcs[vc]->isReady(curTick()) &&
                    m_out_vc_state[vc]->has_credit()) {

                    bool is_candidate_vc = true;
                    int vc_base = t_vnet * m_vc_per_vnet;

                    if (m_net_ptr->isVNetOrdered(t_vnet)) {
                        for (int vc_offset = 0; vc_offset < m_vc_per_vnet;
                             vc_offset++) {
                            int t_vc = vc_base + vc_offset;
                            if (m_ni_out_vcs[t_vc]->isReady(curTick())) {
                                if (m_ni_out_vcs_enqueue_time[t_vc] <
                                    m_ni_out_vcs_enqueue_time[vc]) {
                                    is_candidate_vc = false;
                                    break;
                                }
                            }
                        }
                    }
                    if (!is_candidate_vc)
                        continue;

                    // Update the round robin arbiter
                    oPort->vcRoundRobin(vc);

                    m_out_vc_state[vc]->decrement_credit();
                    // Just removing the flit
                    flit *t_flit = m_ni_out_vcs[vc]->getTopFlit();
                    t_flit->set_time(clockEdge(Cycles(1)));
                    scheduleFlit(t_flit);

                    if (t_flit->get_type() == TAIL_ ||
                       t_flit->get_type() == HEAD_TAIL_) {
                        m_ni_out_vcs_enqueue_time[vc] = Tick(INFINITE_);
                    }
                    goto donePort;
                }
            }
        }
donePort:
        continue;
    }
}

NetworkInterface::InputPort *
NetworkInterface::getInportForVnet(int vnet)
{
    for (auto &iPort : inPorts) {
        if (iPort->isVnetSupported(vnet)) {
            return iPort;
        }
    }

    return NULL;
}

NetworkInterface::OutputPort *
NetworkInterface::getOutportForVnet(int vnet)
{
    for (auto &oPort : outPorts) {
        if (oPort->isVnetSupported(vnet)) {
            return oPort;
        }
    }

    return NULL;
}
void
NetworkInterface::scheduleFlit(flit *t_flit)
{
    OutputPort *oPort = getOutportForVnet(t_flit->get_vnet());

    if (oPort) {
        DPRINTF(RubyNetwork, "Scheduling at %s time:%ld flit:%s Message:%s\n",
        oPort->outNetLink()->name(), clockEdge(Cycles(1)),
        *t_flit, *(t_flit->get_msg_ptr()));
        DPRINTF(RubySNI, "Network Interface: Scheduling at %s time:%ld flit:%s Message:%s\n", 
        oPort->outNetLink()->name(), clockEdge(Cycles(1)),
        *t_flit, *(t_flit->get_msg_ptr()));
        oPort->outFlitQueue()->insert(t_flit);
        oPort->outNetLink()->scheduleEventAbsolute(clockEdge(Cycles(1)));
        return;
    }

    fatal("No output port found for vnet:%d\n", t_flit->get_vnet());
    return;
}

int
NetworkInterface::get_vnet(int vc)
{
    for (int i = 0; i < m_virtual_networks; i++) {
        if (vc >= (i*m_vc_per_vnet) && vc < ((i+1)*m_vc_per_vnet)) {
            return i;
        }
    }
    fatal("Could not determine vc");
}


// Wakeup the NI in the next cycle if there are waiting
// messages in the protocol buffer, or waiting flits in the
// output VC buffer.
// Also check if we have to reschedule because of a clock period
// difference.
void
NetworkInterface::checkReschedule()
{
    for (const auto& it : inNode_ptr) {
        if (it == nullptr) {
            continue;
        }

        while (it->isReady(clockEdge())) { // Is there a message waiting
            scheduleEvent(Cycles(1));
            return;
        }
    }

    for (int vc = 0; vc < m_num_vcs; vc++) {
        if (m_ni_out_vcs[vc]->isReady(clockEdge(Cycles(1)))) {
            scheduleEvent(Cycles(1));
            return;
        }
    }

    // Check if any input links have flits to be popped.
    // This can happen if the links are operating at
    // a higher frequency.
    for (auto &iPort : inPorts) {
        NetworkLink *inNetLink = iPort->inNetLink();
        if (inNetLink->isReady(curTick())) {
            scheduleEvent(Cycles(1));
            return;
        }
    }

    for (auto &oPort : outPorts) {
        CreditLink *inCreditLink = oPort->inCreditLink();
        if (inCreditLink->isReady(curTick())) {
            scheduleEvent(Cycles(1));
            return;
        }
    }
}

void
NetworkInterface::print(std::ostream& out) const
{
    out << "[Network Interface]";
}

uint32_t
NetworkInterface::functionalWrite(Packet *pkt)
{
    uint32_t num_functional_writes = 0;
    for (unsigned int i  = 0; i < m_num_vcs; ++i) {
        num_functional_writes += m_ni_out_vcs[i]->functionalWrite(pkt);
    }

    for (auto &oPort: outPorts) {
        num_functional_writes += oPort->outFlitQueue()->functionalWrite(pkt);
    }
    return num_functional_writes;
}

NetworkInterface *
GarnetNetworkInterfaceParams::create()
{
    return new NetworkInterface(this);
}

//A return of "true" means that the message should be broadcasted as it is
//A return of "false" means that the broadcast message should be dropped and
//instead a new response packet needs to be created directed towards the original
//requestor core
bool NetworkInterface::apu_check_NI(MsgPtr new_msg_ptr, NodeID destID){
    
    Addr addr_check = new_msg_ptr->getaddr();
    DPRINTF(RubySNI, "NI_SNI: APU_CHECK: DestID:%0d; Message:%0s\n", destID, *new_msg_ptr);  
    
    //THe chiplet_id is being extracted from the
    //L2Cahce-0/1/2/3... NodeID number Check getRequestor()
    //in RequestMsg/ResponseMsg and then the MachineID structure
    //Since each chiplet has a single L2 Cache,
    //the node number of L2 cache will be equal to the Chiplet ID
    
    //unsigned int requestor_id = -1;
    //unsigned int responder_id = -1;
    //unsigned int responder_id = -1;
    unsigned int chiplet_id = -1;

    chiplet_id = destID/8;

    /* 
    if (t_flit->get_vnet() == 2){
        DPRINTF(RubySNI, "SNI: APU_CHECK: requestor_id obtained\n");    
        requestor_id = (unsigned int)((t_flit->get_msg_ptr())  
                       ->getRequestor()).getNum();
        chiplet_id = requestor_id%8;
    } 
    else if(t_flit->get_vnet() == 4){
        DPRINTF(RubySNI, "SNI: APU_CHECK: responder_id obtained\n");     
        responder_id = (unsigned int)((t_flit->get_msg_ptr())   
                      ->getSender()).getNum();             
        chiplet_id = responder_id%8;
    } 
    */

    //unsigned int requestor_id = (unsigned int)((t_flit->get_msg_ptr())
    //                          ->getRequestor()).getNum();
    
    if (chiplet_id == -1 || (chiplet_id >= 8)){
        assert(false);           
    }

    int apu_policy_size = system->APU_Table.size();
    
    if (chiplet_id != -1) {
    //unsigned int request_type = (t_flit->get_msg_ptr())->getreqtype();
        DPRINTF(RubySNI, "TJ SNI@DIR: Chiplet_ID:%d, Addr:%x, APU_size:%d\n", chiplet_id, addr_check, apu_policy_size);  
        
        for (int i=0; i<apu_policy_size; i++){
         
        //Read type - GETX and GETS
        //Write type -
            if ((addr_check>=system->APU_Table[i].apuaddr_start_page*4096)
               && ((addr_check<=((system->APU_Table[i].apuaddr_end_page+1)*4096)-1))){
                  if(chiplet_id==0){
                      if ((system->APU_Table[i].apumid & 0x1) == 0){
                           return false;
                      }
                      else{
                          //This means that this particular Chiplet has access to this cache block
                          //And hence the broadcast to this chiplet should be sent 
                          return true;
                      }
                  }
                  else if(chiplet_id==1){                                    
                      if ((system->APU_Table[i].apumid & 0x2) == 0){
                           return false;
                      }else{
                           return true;
                      }
                  }
                  else if(chiplet_id==2){                                    
                      if ((system->APU_Table[i].apumid & 0x4) == 0){
                          return false;  
                      }else{
                          return true;
                      }
                  }    
                  else if(chiplet_id==3){                                    
                      if ((system->APU_Table[i].apumid & 0x8) == 0){
                          return false; 
                      }else{
                          return true;
                      }
                  }    
                  else if(chiplet_id==4){                                    
                      if ((system->APU_Table[i].apumid & 0x10) == 0){
                          return false; 
                      }else{
                          return true; 
                      }
                  }    
                  else if(chiplet_id==5){                                    
                      if ((system->APU_Table[i].apumid & 0x20) == 0){
                           return false; 
                      }else{
                           return true;
                      }
                  }    
                  else if(chiplet_id==6){                                    
                      if ((system->APU_Table[i].apumid & 0x40) == 0){
                           return false; 
                      }else{
                           return true;
                      }
                  }    
                  else if(chiplet_id==7){                                    
                      if ((system->APU_Table[i].apumid & 0x80) == 0){
                          return false; 
                      }else{
                          return true;
                      }
                  }    
                //Pass-Do Nothing!!
                //Add delay/latency for this check
            }
        }
    }
    
    assert(false);
    //Don't think we should ever end up here
    return false;
}

MsgPtr
NetworkInterface::modify_message_sni(MsgPtr new_msg_ptr, NodeID destID){

    DPRINTF(RubySNI, "SNI@DIR: Message:%s; MachineID:%s\n", *new_msg_ptr, new_msg_ptr->getDestination().smallestElement());
   
    std::shared_ptr<ResponseMsg> out_msg = std::make_shared<ResponseMsg>(clockEdge());    
    (*out_msg).m_addr = new_msg_ptr->getaddr(); 

    if(new_msg_ptr->getCohReqType() == 0 || new_msg_ptr->getCohReqType() == 1){
        (*out_msg).m_Type = CoherenceResponseType_ACK;
    }
    (*out_msg).m_Sender = MachineID(MachineType_L1Cache, destID);
    DPRINTF(RubySNI, "SNI@DIR: destionation:%s\n", new_msg_ptr->getRequestor()); 
    ((*out_msg).m_Destination).add(new_msg_ptr->getRequestor());
    (*out_msg).m_Acks = (1);
    (*out_msg).m_SilentAcks = new_msg_ptr->getSilentAcks();
    //#ifndef NDEBUG
    //if (!((((*in_msg_ptr)).m_DirectedProbe == (false)))) {
    //    panic("Runtime Error at MOESI_hammer-cache.sm:804: %s.\n", "assert failure");
    //}
    //#endif
    (*out_msg).m_MessageSize = MessageSizeType_Response_Control;
    (*out_msg).m_InitialRequestTime = new_msg_ptr->getInitialRequestTime();
    (*out_msg).m_ForwardRequestTime = new_msg_ptr->getForwardRequestTime();

    DPRINTF(RubySNI, "SNI@DIR: Modified Message:%s;\n", *out_msg);

    return out_msg;
}


/*
std::vector<NetworkInterface::APU> NetworkInterface::apu_policy_init_NI(){
    std::vector<APU> APU_temp;
     
    const char *homedir; 
    
    if ((homedir = getenv("HOME")) == NULL) {
        homedir = getpwuid(getuid())->pw_dir;
    }
    std::cout<<"Home Directory: "<<homedir<<'\n';   

    std::string filename = std::string(homedir) + "/gem5_garnet2/my_benchmarks/apu_init/apu_init.txt";
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
*/
