/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016,2016 BUET
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Tanvir Ahmed Khan <takhandipu@gmail.com>
 */
#include "ns3/wifi-net-device.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/yans-wifi-phy.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/error-rate-model.h"
#include "ns3/yans-error-rate-model.h"
#include "ns3/ptr.h"
#include "ns3/mobility-model.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/vector.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/nstime.h"
#include "ns3/command-line.h"
#include "ns3/flow-id-tag.h"
#include "ns3/core-module.h"
#include "ns3/delay-jitter-estimation.h"
#include "ns3/pu-model.h"

using namespace ns3;
using namespace std;


NS_LOG_COMPONENT_DEFINE ("Base");

#define MAX_CHANNELS 11

#define BACKUP 0
#define FRAGMENTED 1
#define CHANNEL_DISTRIBUTED 2
#define RADIO_UTILIZATION_RATIO 4
#define CHANNEL_UTILIZATION_RATIO 8

int packetSize = 1024; //packet Size 1KB
double packetInterval = 1.0/128.0; //data injection 128 KBps, 1Mbps
int totalPacketsPerSU = 1024 * 20; //per SU 20 MB data
string txMode = "OfdmRate18Mbps";
uint8_t txPowerLevel = 0;
bool alwaysSwitch=true;

double sourceSinkDistance = 80.0;

double puArrivalMean = 5.0;
double puArrivalBound = 10.0;

double puDepartureMean = 2.0;
double puDepartureBound = 4.0;

double endTime = 50.0;

double width = 500.0;

double senseTime = 0.01;
double switchTime = 0.005;
double transmissionTime = 0.001;

unsigned int mode = 0;
unsigned int seed=1;

double wakeUpRadioProbability = 0.2;

Ptr<UniformRandomVariable> positionAllocator = CreateObject<UniformRandomVariable>();

Ptr<UniformRandomVariable> randomChannelAllocator = CreateObject<UniformRandomVariable>();

Ptr<UniformRandomVariable> probabilisticChannelAllocator = CreateObject<UniformRandomVariable>();

Ptr<UniformRandomVariable> randomRadioAllocator = CreateObject<UniformRandomVariable>();

Ptr<ExponentialRandomVariable> primaryUserArrivalAllocator = CreateObject<ExponentialRandomVariable> ();
Ptr<ExponentialRandomVariable> primaryUserDepartureAllocator = CreateObject<ExponentialRandomVariable> ();

Ptr<YansWifiChannel> channels[MAX_CHANNELS];

Ptr<PUModel> puModel;

int SUNodeCount = 0;

bool isBackup()
{
    unsigned backupBit=mode&1;
    if(backupBit==0)return true;
    return false;
}

bool isFragmented()
{
    return !isBackup();
}

class MySUNode
{
public:
    int nodeId;
    int numberOfRadios;
    Ptr<YansWifiPhy> *m_txes;
    Ptr<YansWifiPhy> *m_rxes;
    Ptr<MobilityModel> posTx;
    Ptr<MobilityModel> posRx;
    int *currentChannel;
    uint32_t *m_flowIds;
    int *packetsQueuedPerRadio;
    int *packetsSentPerRadio;
    int *packetsReceivedPerRadio;
    int packetsSentPerChannel[MAX_CHANNELS+1];
    int packetsReceivedPerChannel[MAX_CHANNELS+1];
    Time delaySum;
    Time timeFirstRxPacket;
    Time timeLastRxPacket;
    bool isFirst;
    double perNodeAvgThroughput;
    double perPacketAvgDelay;
    double packetDropRatio;
    int totalPacketsQueued;
    int totalPacketsSent;
    int totalPacketsReceived;
    int maxPacketsReceived;
    int effectivePacketsReceived;
    int channelAllcoation[MAX_CHANNELS];
    bool *radioOnStatus;
    void transmitPacketInChannel(int radioIndex, Ptr<Packet> p)
    {
        /**/
        m_txes[radioIndex]->SendPacket (p, WifiMode (txMode), WIFI_PREAMBLE_SHORT, txPowerLevel);
        packetsSentPerRadio[radioIndex]++;
        packetsSentPerChannel[currentChannel[radioIndex]]++;
    }
    void startSwitching(int radioIndex, Ptr<Packet> p)
    {
        if(alwaysSwitch)
        {
            vector<int> freeChannelIndexes;
            for(int i=0; i<MAX_CHANNELS; i++)
            {
                if(channelAllcoation[i]==true)continue;
                freeChannelIndexes.push_back(i);
            }
            if(freeChannelIndexes.size()==0)
            {                
                //Simulator::Schedule (Seconds (switchTime), &MySUNode::startSwitching, this, radioIndex, p);
                radioOnStatus[radioIndex]=false;
                return;
            }
            
            double *sentToReceivedRatios = new double[freeChannelIndexes.size()];
            double total=0.0;
            for(unsigned int i = 0; i < freeChannelIndexes.size(); i++)
            {
                sentToReceivedRatios[i]=(1.0+packetsSentPerChannel[freeChannelIndexes[i]])/(1.0+packetsReceivedPerChannel[freeChannelIndexes[i]]);
                total+=sentToReceivedRatios[i];
            }
            for(unsigned int i = 0; i < freeChannelIndexes.size(); i++)
            {
                sentToReceivedRatios[i]/=total;
            }
            double randValue = probabilisticChannelAllocator->GetValue();
            double current = 0.0;
            int selectedChannelIndex = 0;
            for(unsigned int i = 0; i < freeChannelIndexes.size(); i++)
            {
                current += sentToReceivedRatios[i];
                if(randValue < current)
                {
                    selectedChannelIndex=i;
                    break;
                }
            }
            int changedChannelIndex = freeChannelIndexes[selectedChannelIndex];
            
            char debugString[80];
            sprintf(debugString, "Node %d Radio %d is switching to channel %d",nodeId,radioIndex,changedChannelIndex);
            NS_LOG_DEBUG(debugString);
            
            bool notused=true;
            for(int i=0; i<numberOfRadios; i++)
            {
                if(i==radioIndex)continue;
                if(currentChannel[i]==currentChannel[radioIndex])notused=false;
            }
            if(notused)channelAllcoation[currentChannel[radioIndex]]=false;

            currentChannel[radioIndex] = changedChannelIndex;
            m_txes[radioIndex]->SetChannel(channels[changedChannelIndex]);
            m_rxes[radioIndex]->SetChannel(channels[changedChannelIndex]);
            channelAllcoation[changedChannelIndex]=true;
        }
        Simulator::Schedule (Seconds (switchTime), &MySUNode::startSensing, this, radioIndex, p);
    }
    void startSensing(int radioIndex, Ptr<Packet> p)
    {
        radioOnStatus[radioIndex]=true;
        bool senseResult=false;
        bool txSenseResult = puModel->IsPuActive(Now(),Time(senseTime),posTx->GetPosition().x,posTx->GetPosition().y,currentChannel[radioIndex]);
        bool rxSenseResult = puModel->IsPuActive(Now(),Time(senseTime),posRx->GetPosition().x,posRx->GetPosition().y,currentChannel[radioIndex]);
        senseResult = txSenseResult || rxSenseResult;
        if(senseResult==true)
        {
            //PU is present
            //schedule switching
            Simulator::Schedule (Seconds (senseTime), &MySUNode::startSwitching, this, radioIndex, p);
        } else {
            //PU is not present
            //schedule transmission
            Simulator::Schedule (Seconds(senseTime), &MySUNode::transmitPacketInChannel, this, radioIndex, p);
        }
    }
    void sendPacketInRadio(int radioIndex)
    {
        radioOnStatus[radioIndex]=true;
        Ptr<Packet> p = Create<Packet> (packetSize);
        p->AddByteTag (FlowIdTag (m_flowIds[radioIndex]));
        
        DelayJitterEstimationTimestampTag timeTag;
        p->AddPacketTag(timeTag);
        
        packetsQueuedPerRadio[radioIndex]++;
        
        if(isFirst == true)
        {
            timeFirstRxPacket = Now();
            isFirst = false;
        }
        
        Simulator::Schedule (Seconds (0.0), &MySUNode::startSensing, this, radioIndex, p);
    }
    DelayJitterEstimationTimestampTag* getDelayJitterEstimationTimestampTag(Ptr<Packet> p)
    {
        PacketTagIterator i = p->GetPacketTagIterator ();
        DelayJitterEstimationTimestampTag *tag;
        while(i.HasNext ())
        {
            PacketTagIterator::Item item = i.Next ();
            //NS_ASSERT (item.GetTypeId ().HasConstructor ());
            if(!item.GetTypeId ().HasConstructor ())break;
            Callback<ObjectBase *> constructor = item.GetTypeId ().GetConstructor ();
            //NS_ASSERT (!constructor.IsNull ());
            if(constructor.IsNull ())break;
            ObjectBase *instance = constructor ();
            tag = dynamic_cast<DelayJitterEstimationTimestampTag *> (instance);
            //NS_ASSERT (tag != 0);
            if(tag==0)break;
            item.GetTag (*tag);
            break;
        }
        return tag;
    }
    void receivePacketInRadio(Ptr<Packet> p, double snr, WifiMode mode, enum WifiPreamble preamble)
    {
        FlowIdTag tag;
        p->FindFirstMatchingByteTag (tag);
        
        for(int i = 0; i < numberOfRadios; i++)
        {
            if(tag.GetFlowId () == m_flowIds[i])
            {
                packetsReceivedPerRadio[i]++;
                packetsReceivedPerChannel[currentChannel[i]]++;
                
                DelayJitterEstimationTimestampTag *timeTag = getDelayJitterEstimationTimestampTag(p);
                if(timeTag==0)
                {
                    //
                } else {
                    Time txTime = timeTag->GetTxTime();
                    delaySum += Now()-txTime;
                    //
                    timeLastRxPacket = Now();
                }
                
                break;
            }
        }
    }
    MySUNode(int nRadios=1)
    {
        this->nodeId = SUNodeCount++;
        for(int i=0; i<MAX_CHANNELS; i++)
        {
            channelAllcoation[i]=false;
            packetsSentPerChannel[i]=0;
            packetsReceivedPerChannel[i]=0;
        }
        char debugString[80];
        sprintf(debugString, "Creating %d-th SU node with %d radios.",this->nodeId,nRadios);
        NS_LOG_DEBUG(debugString);
        numberOfRadios=nRadios;
        m_txes = new Ptr<YansWifiPhy>[numberOfRadios];
        m_rxes = new Ptr<YansWifiPhy>[numberOfRadios];
        currentChannel = new int[numberOfRadios];
        double x = positionAllocator->GetValue();
        double y = positionAllocator->GetValue();
        posTx = CreateObject<ConstantPositionMobilityModel> ();
        posTx->SetPosition (Vector (x, y, 0.0));
        posRx = CreateObject<ConstantPositionMobilityModel> ();
        posRx->SetPosition (Vector (x+sourceSinkDistance, y, 0.0));
        
        Ptr<ErrorRateModel> error = CreateObject<YansErrorRateModel> ();
        
        
        //
        
        for(int i = 0; i < numberOfRadios; i++)
        {
            Ptr<YansWifiPhy> tx = CreateObject<YansWifiPhy> ();
            Ptr<YansWifiPhy> rx = CreateObject<YansWifiPhy> ();
            tx->SetErrorRateModel(error);
            rx->SetErrorRateModel(error);
            
            int initialChannelIndex;
            
            vector<int> freeChannelIndexes;
            for(int j=0; j<MAX_CHANNELS; j++)
            {
                if(channelAllcoation[j]==true)continue;
                freeChannelIndexes.push_back(j);
            }
            
            if(freeChannelIndexes.size()==0)
            {                
                initialChannelIndex = randomChannelAllocator->GetInteger() % MAX_CHANNELS;
            } else {
                int randValue = randomChannelAllocator->GetInteger() % freeChannelIndexes.size();
                initialChannelIndex = freeChannelIndexes[randValue];
            }
            
            char debugString[80];
            sprintf(debugString, "Node %d Radio %d got channel %d",this->nodeId,i, initialChannelIndex);
	        NS_LOG_DEBUG(debugString);
            
            currentChannel[i] = initialChannelIndex;
            channelAllcoation[initialChannelIndex]=true;
            tx->SetChannel(channels[initialChannelIndex]);
            rx->SetChannel(channels[initialChannelIndex]);
            tx->SetMobility(posTx);
            rx->SetMobility(posRx);
            
            m_txes[i]=tx;
            m_rxes[i]=rx;
        }
        
        packetsQueuedPerRadio = new int[numberOfRadios];
        packetsSentPerRadio = new int[numberOfRadios];
        packetsReceivedPerRadio = new int[numberOfRadios];
        
        m_flowIds = new uint32_t[numberOfRadios];
        
        for(int i = 0; i < numberOfRadios; i++)
        {
            packetsQueuedPerRadio[i] = 0;
            packetsSentPerRadio[i] = 0;
            packetsReceivedPerRadio[i] = 0;
            
            m_flowIds[i] = FlowIdTag::AllocateFlowId ();
        }
        
        delaySum=Time(0.0);

        perNodeAvgThroughput=0.0;
        perPacketAvgDelay=0.0;
        packetDropRatio=0.0;
        
        totalPacketsQueued=0;
        totalPacketsSent=0;
        totalPacketsReceived=0;
        maxPacketsReceived=0;
        
        for(int i = 0; i < numberOfRadios; i++)
        {
            m_rxes[i]->SetReceiveOkCallback ( MakeCallback ( &MySUNode::receivePacketInRadio, this ) );
        }
        
        radioOnStatus=new bool[numberOfRadios];
        for(int i = 0; i < numberOfRadios; i++)
        {
            radioOnStatus[i]=true;
        }
        
        isFirst = true;
        timeFirstRxPacket = Now();
        timeLastRxPacket = Now();
        
        /*if(isBackup())startBackup();
        else startFragmented();*/
        //
        startFeedback();

    }
    void startBackup()
    {
        double currentTime = 0.0;
        for(int i = 0; i < totalPacketsPerSU; i++)
        {
            for(int j = 0; j < numberOfRadios; j++)
            {
                Simulator::Schedule (Seconds (currentTime), &MySUNode::sendPacketInRadio, this, j);
            }
            currentTime += packetInterval;
        }
    }
    void startFragmented()
    {
        double currentTime = 0.0;
        int effectivePacketsPerRadio=totalPacketsPerSU/numberOfRadios;
        for(int i = 0; i < effectivePacketsPerRadio; i++)
        {
            for(int j = 0; j < numberOfRadios; j++)
            {
                Simulator::Schedule (Seconds (currentTime), &MySUNode::sendPacketInRadio, this, j);
            }
            currentTime += packetInterval;
        }
    }
    void startFeedback()
    {
        double currentTime = 0.0;
        Simulator::Schedule (Seconds (currentTime), &MySUNode::giveFeedback, this, totalPacketsPerSU);
    }
    void giveFeedback(int packetsToSend)
    {
        if(packetsToSend == 0)
        {
            return;
        }
        double *sentToQueuedRatios=new double[numberOfRadios];
        double total=0.0;
        for(int i=0; i<numberOfRadios; i++)
        {
            sentToQueuedRatios[i]=(1.0+packetsSentPerRadio[i])/(1.0+packetsQueuedPerRadio[i]);
            if(radioOnStatus[i]==false) {
                sentToQueuedRatios[i]*=wakeUpRadioProbability;
            }
            total+=sentToQueuedRatios[i];
        }
        for(int i=0; i<numberOfRadios; i++)
        {
            sentToQueuedRatios[i]/=total;
        }
        double randValue = randomRadioAllocator->GetValue();
        double current = 0.0;
        
        int selectedRadioIndex = 0;
        for(int i=0; i<numberOfRadios; i++)
        {
            current += sentToQueuedRatios[i];
            if(randValue < current)
            {
                selectedRadioIndex=i;
                break;
            }
        }
        Simulator::Schedule (Seconds (0.0), &MySUNode::sendPacketInRadio, this, selectedRadioIndex);
        Simulator::Schedule (Seconds (packetInterval), &MySUNode::giveFeedback, this, packetsToSend-1);
    }
    void saveStatistics()
    {
        for(int i = 0; i < numberOfRadios; i++)
        {
            totalPacketsQueued += packetsQueuedPerRadio[i];
            totalPacketsSent += packetsSentPerRadio[i];
            totalPacketsReceived += packetsReceivedPerRadio[i];
            if(packetsReceivedPerRadio[i]>maxPacketsReceived)
            {
                maxPacketsReceived = packetsReceivedPerRadio[i];
            }
        }
        double delayInSecond = delaySum.GetDouble() /1000000000;
        if(totalPacketsReceived!=0)perPacketAvgDelay=(delayInSecond/totalPacketsReceived);
        if(totalPacketsSent!=0)packetDropRatio=(1.0*(totalPacketsSent-totalPacketsReceived)/totalPacketsSent);
        effectivePacketsReceived=totalPacketsReceived;
        if(isBackup())effectivePacketsReceived=maxPacketsReceived;
        if(timeLastRxPacket==timeFirstRxPacket)
        {
            perNodeAvgThroughput=0.0;
        }
        else
        {
            Time difference = timeLastRxPacket - timeFirstRxPacket;//in ns
            double diff = difference.GetDouble()/1000000000;//in s
            //packetCount = effectivePacketsReceived;
            double byteCount = effectivePacketsReceived * packetSize;
            double bitps = byteCount * 8;
            perNodeAvgThroughput = bitps/diff;
        }
    }
    void printSimulationResults()
    {
        saveStatistics();
        //
        NS_LOG_DEBUG("Node "<<nodeId);
        for(int i = 0; i < numberOfRadios; i++)
        {
            NS_LOG_DEBUG("Radio "<<i);
            NS_LOG_DEBUG("Packets Sent "<<packetsSentPerRadio[i]);
            NS_LOG_DEBUG("Packets Received "<<packetsReceivedPerRadio[i]);
        }
        NS_LOG_DEBUG("Delay Sum (in seconds) "<<delaySum/1000000000);
        NS_LOG_DEBUG("Per Node Avg. Throughput (Mbps) "<<perNodeAvgThroughput/(1024*1024));
        NS_LOG_DEBUG("Per Packet Avg. Delay (Seconds) "<<perPacketAvgDelay);
        NS_LOG_DEBUG("Packet Drop Ratio "<<packetDropRatio);
    }
};

class MyExperiment
{
public:
    Ptr<WifiPhy> m_puTxes[MAX_CHANNELS];
    int numberOfSUs;
    int numberOfRadios;
    MySUNode *sus[20];
    double avgNetworkThroughput;//bps
    double perNodeAvgThroughput;//bps
    double perPacketAvgDelay;
    double packetDropRatio;
    void sendPacketInPU(int puIndex, Time endTime)
    {
        if(Now()>endTime){
            double nextTime = primaryUserArrivalAllocator->GetValue();
            Simulator::Schedule (Seconds (nextTime), &MyExperiment::startPUActivity, this, puIndex);
        } else {
            Ptr<Packet> p = Create<Packet> (packetSize);
            m_puTxes[puIndex]->SendPacket (p, WifiMode (txMode), WIFI_PREAMBLE_SHORT, txPowerLevel);
            
            char debugString[80];
            sprintf(debugString, "PU %d is sending a packet",puIndex);
            NS_LOG_DEBUG(Now()/1000000000<<" "<<debugString);
            
            Simulator::Schedule (Seconds (packetInterval), &MyExperiment::sendPacketInPU, this, puIndex, endTime);
        }
    }
    void startPUActivity(int puIndex)
    {
        double nextTime = primaryUserDepartureAllocator->GetValue();
        Simulator::Schedule (Seconds (0.0), &MyExperiment::sendPacketInPU, this, puIndex, Now()+Seconds(nextTime));
    }
    MyExperiment(int numberOfSUs=4, int numberOfRadios=1)
    {
        avgNetworkThroughput=0.0;//bps
        perNodeAvgThroughput=0.0;//bps
        perPacketAvgDelay=0.0;
        packetDropRatio=0.0;
        SUNodeCount = 0;
        this->numberOfRadios=numberOfRadios;
        positionAllocator->SetAttribute ("Min", DoubleValue (0.0));
        positionAllocator->SetAttribute ("Max", DoubleValue (width));
        randomChannelAllocator->SetAttribute ("Min", DoubleValue (1));
        randomChannelAllocator->SetAttribute ("Max", DoubleValue (MAX_CHANNELS));
        probabilisticChannelAllocator->SetAttribute ("Min", DoubleValue (0.0));
        probabilisticChannelAllocator->SetAttribute ("Max", DoubleValue (1.0));
        
        
        randomRadioAllocator->SetAttribute ("Min", DoubleValue (0.0));
        randomRadioAllocator->SetAttribute ("Max", DoubleValue (1.0));
        
        primaryUserArrivalAllocator->SetAttribute ("Mean", DoubleValue (puArrivalMean));
        primaryUserArrivalAllocator->SetAttribute ("Bound", DoubleValue (puArrivalBound));
        primaryUserDepartureAllocator->SetAttribute ("Mean", DoubleValue (puDepartureMean));
        primaryUserDepartureAllocator->SetAttribute ("Bound", DoubleValue (puDepartureBound));
        
        
        puModel = CreateObject<PUModel>();
        std::string map_file = "newPUTrace.txt";//"map_PUs_multiple.txt";//
        puModel->SetPuMapFile((char*)map_file.c_str());
        
        for(int i=0; i<MAX_CHANNELS; i++)
        {
            channels[i] = CreateObject<YansWifiChannel> ();
            channels[i]->SetPropagationDelayModel(CreateObject<ConstantSpeedPropagationDelayModel> ());
            channels[i]->SetPropagationLossModel ( CreateObject<LogDistancePropagationLossModel> () );
        }
        
        Ptr<ErrorRateModel> error = CreateObject<YansErrorRateModel> ();
        for(int i = 0; i < MAX_CHANNELS; i++)
        {
            Ptr<YansWifiPhy> tx = CreateObject<YansWifiPhy> ();
            tx->SetErrorRateModel(error);
            tx->SetChannel(channels[i]);
            
            Ptr<MobilityModel> posTx = CreateObject<ConstantPositionMobilityModel> ();
            posTx->SetPosition (Vector (positionAllocator->GetValue(), positionAllocator->GetValue(), 0.0));
            tx->SetMobility(posTx);
            
            m_puTxes[i]=tx;
            
            //double nextTime = primaryUserArrivalAllocator->GetValue();
            //Simulator::Schedule (Seconds (nextTime), &MyExperiment::startPUActivity, this, i);
        }
        
        
        this->numberOfSUs = numberOfSUs;
        for(int i = 0; i<numberOfSUs; i++)
        {
            sus[i] = new MySUNode(numberOfRadios);
        }
        Simulator::Stop (Seconds (endTime+1.0));
        Simulator::Run ();
        Simulator::Destroy();
        for(int i = 0; i<numberOfSUs; i++)
        {
            sus[i]->printSimulationResults();
        }
        saveStatistics();
        printSimulationResults();
    }
    void saveStatistics()
    {
        int totalPacketsQueued = 0;
        int totalPacketsReceived = 0;
        for(int i=0; i<numberOfSUs; i++)
        {
            totalPacketsQueued += sus[i]->totalPacketsQueued;
            totalPacketsReceived += sus[i]->totalPacketsReceived;
        }
        double drop = (totalPacketsQueued-totalPacketsReceived)*1.0;
        if(totalPacketsQueued==0)
        {
            packetDropRatio = 0.0;
        } else {
            packetDropRatio = drop/totalPacketsQueued;
        }
        for(int i=0; i<numberOfSUs; i++)
        {
            avgNetworkThroughput += sus[i]->perNodeAvgThroughput;
        }
        
        if(numberOfSUs!=0)perNodeAvgThroughput=(avgNetworkThroughput/(numberOfSUs*2));
        
        Time delaySum = Seconds(0.0);
        for(int i = 0; i<numberOfSUs; i++)
        {
            delaySum += sus[i]->delaySum;
        }
        if(totalPacketsReceived==0)
        {
            perPacketAvgDelay = 0.0;
        } else {
            double delayInSecond = delaySum.GetDouble() /1000000000;
            perPacketAvgDelay=(delayInSecond/totalPacketsReceived);
        }
        
        NS_LOG_DEBUG("Network performance matrics");
        NS_LOG_DEBUG("Avg. Network Throughput (Mbps) "<<avgNetworkThroughput/(1024*1024));
        NS_LOG_DEBUG("Per Packet Avg. Delay (Seconds) "<<perPacketAvgDelay);
        NS_LOG_DEBUG("Per Node Avg. Throughput (Mbps) "<<perNodeAvgThroughput/(1024*1024));
        NS_LOG_DEBUG("Packet Drop Ratio "<<packetDropRatio);
    }
    void printSimulationResults()
    {
        cout<<numberOfSUs*2<<", ";
        cout<<numberOfRadios<<", ";
        cout<<avgNetworkThroughput/(1024*1024)<<", ";
        cout<<perPacketAvgDelay<<", ";
        cout<<perNodeAvgThroughput/(1024*1024)<<", ";
        cout<<packetDropRatio<<", ";
        cout<<seed<<", ";
        cout<<(1.0/packetInterval)<<", ";
        cout<<endl;
    }
};

int main (int argc, char *argv[])
{
    mode = 1;
    int numNodes = 6;
    int numRadios = 1;
    seed=1;
    double packetPerSecond=128.0;

    CommandLine cmd;

    cmd.AddValue ("numNodes", "Number of nodes", numNodes);
    cmd.AddValue ("numRadios", "Number of nodes", numRadios);
    cmd.AddValue ("endTime", "Total Simulation Time", endTime);
    cmd.AddValue ("seed", "Simulation seed", seed);
    cmd.AddValue ("packetPerSecond", "Packet Per Second", packetPerSecond);

    cmd.Parse (argc, argv);
    
    packetInterval = 1.0/packetPerSecond;

    SeedManager::SetSeed(seed);
    
    MyExperiment ex(numNodes,numRadios);
    return 0;
}
