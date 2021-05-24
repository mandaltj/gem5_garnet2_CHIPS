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


#include "mem/ruby/network/garnet2.0/NetworkLink.hh"
#include "mem/ruby/protocol/CoherenceRequestType.hh"
#include "mem/ruby/protocol/ResponseMsg.hh"
#include "mem/ruby/network/garnet2.0/CreditLink.hh"
#include "mem/ruby/protocol/NetDest.hh"

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h> 

//std::vector<NetworkLink::APU> NetworkLink::APU_Table = apu_policy_init_NL();

NetworkLink::NetworkLink(const Params *p)
    : ClockedObject(p), Consumer(this), 
      system(p->system), 
      m_id(p->link_id),
      m_type(NUM_LINK_TYPES_),
      m_latency(p->link_latency),
      linkBuffer(new flitBuffer()), link_consumer(nullptr),
      link_srcQueue(nullptr), m_link_utilized(0),
      m_vc_load(p->vcs_per_vnet * p->virt_nets),
      m_virt_nets(p->virt_nets)
{
    int num_vnets = (p->supported_vnets).size();
    assert(num_vnets > 0);
    mVnets.resize(num_vnets);
    bitWidth = p->width;
    for (int i = 0; i < num_vnets; i++) {
        mVnets[i] = p->supported_vnets[i];
    }
    enSNI = p->sni;

}

NetworkLink::~NetworkLink()
{
    delete linkBuffer;
}

void
NetworkLink::setLinkConsumer(Consumer *consumer)
{
    link_consumer = consumer;
}

void
NetworkLink::setVcsPerVnet(uint32_t consumerVcs)
{
    m_vc_load.resize(m_virt_nets * consumerVcs);
}

void
NetworkLink::setSourceQueue(flitBuffer *srcQueue, ClockedObject *srcClockObj)
{
    link_srcQueue = srcQueue;
    src_object = srcClockObj;
}

void
NetworkLink::wakeup()
{
    DPRINTF(RubyNetwork, "Woke up to transfer from %s to %s\n",
    src_object->name(), link_consumer->getName());
    assert(curTick() == clockEdge());
    
    bool apu_result = false;

    if (link_srcQueue->isReady(curTick())) {
        flit *t_flit = link_srcQueue->getTopFlit();
        DPRINTF(RubyNetwork, "Transmission will finish at %ld :%s\n",
                clockEdge(m_latency), *t_flit);
        //DPRINTF(RubySNI, "Transmission will finish at %ld :%s\n",
        //        clockEdge(m_latency), *t_flit);    

        //DPRINTF(RubySNI, "Flit Type: %s, VNET Type: %s\n", t_flit->get_type(), t_flit->get_vnet());    
        //Added by TJ              
         
        //DPRINTF(RubySNI, "SNI: Link ID:%d; enSNI:%0d\n", m_id, enSNI); 
        //DPRINTF(RubySNI, "SNI: Before SNI\n"); 
        if (enSNI && (t_flit->get_type()!=CREDIT_)) {
            //TODO:Check whether the flit passes security check or not
            DPRINTF(RubySNI, "Flit Type: %s, VNET Type: %s\n", t_flit->get_type(), t_flit->get_vnet());     
            DPRINTF(RubySNI, "SNI: Entered SNI\n"); 
            DPRINTF(RubySNI, "SNI: Message:%s\n", *(t_flit->get_msg_ptr()));  
            DPRINTF(RubySNI, "Request/Response Type:%d\n", (t_flit->get_msg_ptr())->getReqRespType());    
           
            //Update: We want to do ResponseChecks as well
            //DO apu_check if only it is aof request type
            //We dont care about response types 
            if((t_flit->get_msg_ptr())->getReqRespType() == 0) {
                //DPRINTF(RubySNI, "SNI: Request_Type:%0d; SNI_Type=%0d\n", (t_flit->get_msg_ptr())->getCohReqType(), sni_type);  
                if((t_flit->get_msg_ptr())->getCohReqType() != 0 &&
                   (t_flit->get_msg_ptr())->getCohReqType() != 1 &&     
                   (t_flit->get_msg_ptr())->getCohReqType() != 2 &&      
                   (t_flit->get_msg_ptr())->getCohReqType() != 3 &&     
                   (t_flit->get_msg_ptr())->getCohReqType() != 4 ){  
                    assert(false);
                } 
            }
            else{
                DPRINTF(RubySNI, "SNI: Entered APU_Check; Response_Type:%0d\n", (t_flit->get_msg_ptr())->getCohRespType());    
            }
            
            if((t_flit->get_msg_ptr())->getReqRespType() == 0) {
                apu_result = apu_check(t_flit); 
            } else {
                //For any other type of message other than Request Msg
                //we will not check it
                apu_result = true;
            }

            DPRINTF(RubySNI, "SNI: APU Check Done\n");      
            if (apu_result == false ){
                //Machine check failure
                DPRINTF(RubySNI, "SNI: Test Failed\n"); 
                assert(false);                             
            }else{
                DPRINTF(RubySNI, "SNI: Test Passed\n");  
                //Do nothing, continue
            }
            DPRINTF(RubySNI, "SNI: Leaving SNI\n");  
        }
        //DPRINTF(RubySNI, "SNI: After SNI\n");   
        
        if (m_type != NUM_LINK_TYPES_) {
            // Only for assertions and debug messages
            assert(t_flit->m_width == bitWidth);
            assert((std::find(mVnets.begin(), mVnets.end(),
                t_flit->get_vnet()) != mVnets.end()) ||
                (std::find(mVnets.begin(), mVnets.end(), -1) != mVnets.end()));
        }
        t_flit->set_time(clockEdge(m_latency));
        linkBuffer->insert(t_flit);
        link_consumer->scheduleEventAbsolute(clockEdge(m_latency));
        m_link_utilized++;
        m_vc_load[t_flit->get_vc()]++;
    }

    if (!link_srcQueue->isEmpty()) {
        scheduleEvent(Cycles(1));;
    }
}

void
NetworkLink::resetStats()
{
    for (int i = 0; i < m_vc_load.size(); i++) {
        m_vc_load[i] = 0;
    }

    m_link_utilized = 0;
}

NetworkLink *
NetworkLinkParams::create()
{
    return new NetworkLink(this);
}

CreditLink *
CreditLinkParams::create()
{
    return new CreditLink(this);
}

uint32_t
NetworkLink::functionalWrite(Packet *pkt)
{
    return linkBuffer->functionalWrite(pkt);
}

/*
//Added by TJ
std::vector<NetworkLink::APU> NetworkLink::apu_policy_init_NL(){
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


bool NetworkLink::apu_check(flit *t_flit){
    Addr addr_check = (t_flit->get_msg_ptr())->getaddr();
    
    //print_APU_Table(); 
    //DPRINTF(RubySNI, "SNI: APU_CHECK: addr_check obtained\n");  
    
    //THe chiplet_id is being extracted from the
    //L2Cahce-0/1/2/3... NodeID number Check getRequestor()
    //in RequestMsg/ResponseMsg and then the MachineID structure
    //Since each chiplet has a single L2 Cache,
    //the node number of L2 cache will be equal to the Chiplet ID
    
    unsigned int requestor_id = -1;
    unsigned int responder_id = -1;
    //unsigned int responder_id = -1;
    unsigned int chiplet_id = -1;

    if ((t_flit->get_msg_ptr())->getReqRespType() == 0){ 
        requestor_id = (unsigned int)((t_flit->get_msg_ptr())   
                   ->getRequestor()).getNum();
        //We have 8 chiplets. CPU 0-7 belong to Chiplet 0. 
        //CPU 8-15 belong to Chiplet 1. etc
        //Hence we need to divide by 8
        chiplet_id = requestor_id/8; 
    
    } else {
        responder_id = (unsigned int)((t_flit->get_msg_ptr())      
                   ->getSender()).getNum();
        chiplet_id = responder_id/8;  
    }
    
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
    
    if (requestor_id == -1 && responder_id == -1){
        assert(false);           
    }

    int apu_policy_size = system->APU_Table.size();
    
    if (requestor_id != -1) {
    //unsigned int request_type = (t_flit->get_msg_ptr())->getreqtype();
        DPRINTF(RubySNI, "TJ: Requestor:%d, Chiplet_ID:%d, Addr:%d, APU_size:%d\n", requestor_id, chiplet_id, addr_check, apu_policy_size);  
        
        for (int i=0; i<apu_policy_size; i++){
         
        //Read type - GETX and GETS
        //Write type -
            if ((addr_check>=system->APU_Table[i].apuaddr_start_page*4096)
               && (addr_check<((system->APU_Table[i].apuaddr_end_page+1)*4096)-1)){
                  DPRINTF(RubySNI, "SNI: Test i=%0d\n",i);    
                  if(chiplet_id==0){
                      if ((system->APU_Table[i].apumid & 0x1) == 0){
                           //DPRINTF(RubySNI, "SNI: Segmentation Fault Debug L2\n");  
                           return false;
                      }
                      else{
                           //DPRINTF(RubySNI, "SNI: Segmentation Fault Debug L3\n"); 
                           return true;
                      }
                  }
                  else if(chiplet_id==1){                                    
                      if ((system->APU_Table[i].apumid & 0x2) == 0){
                           DPRINTF(RubySNI, "SNI: Test Test \n");     
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
                DPRINTF(RubySNI, "SNI: Request Check passed\n");  
                //Pass-Do Nothing!!
                //Add delay/latency for this check
            }
        }
    } 
    /*
    else if (responder_id != -1) {
        DPRINTF(RubySNI, "TJ: Responder:%d, Chiplet_ID:%d, Addr:%d, APU_size:%d\n", responder_id, chiplet_id, addr_check, apu_policy_size);   
        
        for (int i=0; i<apu_policy_size; i++){        
            if(system->APU_Table[i].apumid & (0x1<<chiplet_id)){
                
                if ((addr_check<system->APU_Table[i].apuaddr_start_page*4096)   
                   || (addr_check>((system->APU_Table[i].apuaddr_end_page+1)*4096)-1)){
                    if ((t_flit->get_msg_ptr())->getCohRespType() != 0){
                        //If Response is not "ACK" i.e. Core doesn't have the data for Block
                        DPRINTF(RubySNI, "TJ: i_value:%d, start_addr:%d, end_addr:%d, \n", i, system->APU_Table[i].apuaddr_start_page, system->APU_Table[i].apuaddr_end_page);    
                        return false;
                    } else {
                        //If Response is "ACK" means core doesnt have access
                        //to this address range; which is fine.
                        return true;
                    }
                } else {
                    return true;
                }
            }    
        }        
        DPRINTF(RubySNI, "SNI: Response Check passed\n");  
    }
    */
    //else if (responder_id !=-1){
    //    if((t_flit->get_msg_ptr())->getType() != CoherenceResponseType_ACK){
    //        //assert(false);
    //    }
    //}
    
    //Don't think we should ever end up here
    DPRINTF(RubySNI, "SNI: Test Test Test\n");     
    return false;
}

/*
void NetworkLink::dir_coherence_check(flit *t_flit){
    DPRINTF(RubySNI, "SNI: Message:%s\n", *(t_flit->get_msg_ptr()));  
    if ((t_flit->get_msg_ptr())->getReqRespType() == 0){
        if((t_flit->get_msg_ptr())->getCohReqType() == 0 ||
           (t_flit->get_msg_ptr())->getCohReqType() == 1){
            DPRINTF(RubySNI, "MC SNI: GETS Broadcast Detected\n"); 
            
            //ResponseMsg is of type Message(class)
            //std::shared_ptr converts it into MsgPtr type which is accepted by the flitisizeMessage
            std::shared_ptr<ResponseMsg> out_msg = std::make_shared<ResponseMsg>(clockEdge());    
            (*out_msg).m_addr = (t_flit->get_msg_ptr())->getaddr();  
            (*out_msg).m_Type = CoherenceResponseType_ACK;
            (*out_msg).m_Sender = ((t_flit->get_msg_ptr())->getDestination()).smallestElement();
            (((*out_msg).m_Destination).add((t_flit->get_msg_ptr())->getRequestor()));
            (*out_msg).m_Acks = (1);
            (*out_msg).m_SilentAcks = (t_flit->get_msg_ptr())->getSilentAcks();
            //#ifndef NDEBUG
            //if (!((((*in_msg_ptr)).m_DirectedProbe == (false)))) {
            //    panic("Runtime Error at MOESI_hammer-cache.sm:804: %s.\n", "assert failure");
            //}
            //#endif
            (*out_msg).m_MessageSize = MessageSizeType_Response_Control;
            (*out_msg).m_InitialRequestTime = (t_flit->get_msg_ptr())->getInitialRequestTime();
            (*out_msg).m_ForwardRequestTime = (t_flit->get_msg_ptr())->getForwardRequestTime();
            //This is the only line of code which is not executable from NOC
            //Actually this enques the out_msg; So it's fine to comment this
            //((*m_responseFromCache_ptr)).enqueue(out_msg, clockEdge(), cyclesToTicks(Cycles(m_cache_response_latency)));
            DPRINTF(RubySNI, "SNI: Response Message Created:%s\n", *out_msg);   

            int num_flits = (int) ceil((double) m_network_ptr->MessageSizeType_to_int( 
                out_msg->getMessageSize())/bitWidth);

            DPRINTF(RubySNI, "MC SNI: Message_Size_in_flits:%0d; VNET:%0d\n", num_flits, (t_flit->get_msg_ptr())->getVnet());     
        
        }
    } 
}
*/

void NetworkLink::print_APU_Table(){
    DPRINTF(RubySNI, "=================================================\n");        
    DPRINTF(RubySNI, "APU Table\n");        
    
    for(int i=0; i<4;i++){
        DPRINTF(RubySNI, "%0d - %0d - %0d - %0d - %0d\n",
        system->APU_Table[i].valid,
        system->APU_Table[i].apumid,
        system->APU_Table[i].apuaddr_start_page,
        system->APU_Table[i].apuaddr_end_page,
        system->APU_Table[i].apuperm);              
    }

    DPRINTF(RubySNI, "=================================================\n");         
}
