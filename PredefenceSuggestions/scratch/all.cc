/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 The Boeing Company
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
 */


//
// This script configures two or more CR nodes and sets a "BulkSender" over TCP
// to transmit data from node 0 to node n. The script prints how many bytes
// were received at the sink when the simulation concludes.
// one may increase the number of hops by executing for example:

// ./waf --run "example --numNodes=3"

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/aodv-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/packet-sink.h"

#include "ns3/flow-monitor-module.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

NS_LOG_COMPONENT_DEFINE ("CRTest");

using namespace ns3;
using namespace std;

//typedef Ptr<YansWifiChannel> channelPointer;
//typedef Ptr<WifiNetDevice> devicePointer;
vector<channelPointer> createdChannels;
double endTime = 10.0;

void createChannels(YansWifiChannelHelper wifiChannel, uint32_t numChannels=11)
{
  for(uint32_t iterateVariable = 0; iterateVariable < numChannels; iterateVariable++) {
    channelPointer channel = wifiChannel.Create ();
    createdChannels.push_back(channel);
  }
}

int main (int argc, char *argv[])
{

  SeedManager::SetSeed(1);
  std::string phyMode ("ErpOfdmRate54Mbps");
  int nNodes = 6;
  int nRadios = 1;

  CommandLine cmd;

  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("numNodes", "Number of nodes", nNodes);
  cmd.AddValue ("numRadios", "Number of nodes", nRadios);

  cmd.Parse (argc, argv);

  // disable fragmentation for frames below 2200 bytes
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("2200"));
  // turn off RTS/CTS for frames below 2200 bytes
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("100"));
  // Fix non-unicast data rate to be the same as that of unicast
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",
      StringValue (phyMode));

  NodeContainer c;
  c.Create (nNodes);

  // The below set of helpers will help us to put together the wifi NICs we want
  WifiHelper wifi;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211g);

  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  // This is one parameter that matters when using FixedRssLossModel
  // set it to zero; otherwise, gain will be added
  wifiPhy.Set ("RxGain", DoubleValue (0) );
  // ns-3 supports RadioTap and Prism tracing extensions for 802.11b
  wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11_RADIO);

  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::LogDistancePropagationLossModel");
  //wifiPhy.SetChannel (wifiChannel.Create ());
  createChannels (wifiChannel);
  wifiPhy.SetChannel ( createdChannels[0] );

  // Add a non-QoS upper mac, and disable rate control
  NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
      "DataMode",StringValue (phyMode),
      "ControlMode",StringValue (phyMode));
  // Set it to adhoc mode
  wifiMac.SetType ("ns3::AdhocWifiMac");

  // Position our nodes with 110 m in between
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  double start = 110.0;
  int xs[]={95,105,115,125,145,155};
  int ys[]={70,70,0,0,-70,-70};
  for (int i=0; i<nNodes; i++) {
    positionAlloc->Add (Vector (xs[i], ys[i], 0.0));
    start = start+20;
  }
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  // Read PU file
  Ptr<PUModel> puModel = CreateObject<PUModel>();
  std::string map_file = "map_PUs_multiple.txt";
  puModel->SetPuMapFile((char*)map_file.c_str());
  //Create repository
  Ptr<Repository> repo = CreateObject<Repository>();
  // Install the CR features into the nodes and return the list of devices
  NetDeviceContainer devices = wifi.InstallCR (repo, puModel, mobility, wifiMac, c, createdChannels, nRadios*2+1);

  // For each CR, we have 3 interfaces. Save the first interface in each
  // node which is the CTRL_IFACE in the device_control array.
  NetDeviceContainer devices_control;
  uint32_t diff = nRadios*2 + 1;
  for (uint32_t i=0; i<devices.GetN(); i=i+diff) {
    devices_control.Add(devices.Get(i));
  }

  InternetStackHelper internet;
  AodvHelper aodv;
  internet.SetRoutingHelper(aodv);
  internet.InstallCR (repo, c);

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  // IP addresses are only assigned for control devices
  Ipv4InterfaceContainer i = ipv4.Assign (devices_control);

  //
  // Create a BulkSendApplication and install it on node 0
  //
  uint16_t port = 9;  // well-known echo port number
  int limit = 0;

  /*BulkSendHelper source ("ns3::TcpSocketFactory",
      InetSocketAddress (i.GetAddress (c.GetN()-1), port));*/
  BulkSendHelper source ("ns3::TcpSocketFactory",
      InetSocketAddress (i.GetAddress (1), port));
  // Set the amount of data to send in bytes.  Zero is unlimited.
  source.SetAttribute ("MaxBytes", UintegerValue (limit));
  ApplicationContainer sourceApps = source.Install (c.Get (0));
  sourceApps.Start (Seconds (0.0));
  sourceApps.Stop (Seconds (endTime));

  //
  // Create a PacketSinkApplication and install it on last node in container
  //
  PacketSinkHelper sink ("ns3::TcpSocketFactory",
      InetSocketAddress (Ipv4Address::GetAny (), port));
  //ApplicationContainer sinkApps = sink.Install (c.Get (c.GetN()-1));
  ApplicationContainer sinkApps = sink.Install (c.Get (1));
  sinkApps.Start (Seconds (0.0));
  sinkApps.Stop (Seconds (endTime));

  BulkSendHelper source1 ("ns3::TcpSocketFactory",
      InetSocketAddress (i.GetAddress (3), port));
  // Set the amount of data to send in bytes.  Zero is unlimited.
  source1.SetAttribute ("MaxBytes", UintegerValue (limit));
  ApplicationContainer sourceApps1 = source1.Install (c.Get (2));
  sourceApps1.Start (Seconds (0.0));
  sourceApps1.Stop (Seconds (endTime));

  //
  // Create a PacketSinkApplication and install it on last node in container
  //
  PacketSinkHelper sink1 ("ns3::TcpSocketFactory",
      InetSocketAddress (Ipv4Address::GetAny (), port));
  //ApplicationContainer sinkApps = sink.Install (c.Get (c.GetN()-1));
  ApplicationContainer sinkApps1 = sink1.Install (c.Get (3));
  sinkApps1.Start (Seconds (0.0));
  sinkApps1.Stop (Seconds (endTime));
  
  BulkSendHelper source2 ("ns3::TcpSocketFactory",
      InetSocketAddress (i.GetAddress (5), port));
  // Set the amount of data to send in bytes.  Zero is unlimited.
  source2.SetAttribute ("MaxBytes", UintegerValue (limit));
  ApplicationContainer sourceApps2 = source2.Install (c.Get (4));
  sourceApps2.Start (Seconds (0.0));
  sourceApps2.Stop (Seconds (endTime));

  //
  // Create a PacketSinkApplication and install it on last node in container
  //
  PacketSinkHelper sink2 ("ns3::TcpSocketFactory",
      InetSocketAddress (Ipv4Address::GetAny (), port));
  //ApplicationContainer sinkApps = sink.Install (c.Get (c.GetN()-1));
  ApplicationContainer sinkApps2 = sink2.Install (c.Get (5));
  sinkApps2.Start (Seconds (0.0));
  sinkApps2.Stop (Seconds (endTime));
  
  // Tracing
  wifiPhy.EnablePcap ("/tmp/all", devices);

  // Output what we are doing
  //NS_LOG_UNCOND ("Starting CR test");
  
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll ();
  
  //AsciiTraceHelper ascii;
  //wifiPhy.EnableAsciiAll (ascii.CreateFileStream("all.tr"));

  Simulator::Stop (Seconds (endTime+1.0));
  Simulator::Run ();
  Simulator::Destroy ();

  Ptr<PacketSink> sinkTest = DynamicCast<PacketSink> (sinkApps.Get (0));
  Ptr<PacketSink> sinkTest1 = DynamicCast<PacketSink> (sinkApps1.Get (0));
  Ptr<PacketSink> sinkTest2 = DynamicCast<PacketSink> (sinkApps2.Get (0));
  //std::cout << "Total Bytes Received: " << sinkTest->GetTotalRx () + sinkTest1->GetTotalRx () + sinkTest2->GetTotalRx () << std::endl;
  //TracedCallback<Ptr<const Packet>, const Address &> m_rxTrace;
  //sink1->TraceConnectWithoutContext("Rx", MakeCallback(&RxRcv));

  monitor->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();

  uint64_t bytesTotal = 0;
  double lastRxTime=-1;
  double firstRxTime=-1;

  Time delaySums =Seconds(0.0);
  uint64_t rxPackets=0;
  
  double lostPackets=0;
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
  {
	  //Find the time of first packet arrived to the sink node from all flows
	  if (firstRxTime < 0)
		  firstRxTime = i->second.timeFirstRxPacket.GetSeconds();
	  else
		  if (firstRxTime > i->second.timeFirstRxPacket.GetSeconds() )
			  firstRxTime = i->second.timeFirstRxPacket.GetSeconds();

	  //Find the time of last packet arrived to the sink node from all flows
	  if (lastRxTime < i->second.timeLastRxPacket.GetSeconds() )
		  lastRxTime = i->second.timeLastRxPacket.GetSeconds();

	  //Sum all received bytes of all flows
	  bytesTotal = bytesTotal + i->second.rxBytes;
	  delaySums +=  i->second.delaySum;
	  rxPackets += i->second.rxPackets;
	  lostPackets += i->second.txPackets - i->second.rxPackets;
  }

  /*std::cout << "Num clients = " << nNodes << " "
		  << "Avg throughput = "
		  << bytesTotal*8/(lastRxTime-firstRxTime)/1024
		  << " kbits/sec" << std::endl;

  std::cout << "Num clients = " << nNodes << " "
		  << "Avg Delay = "
		  << delaySums / rxPackets / 1000000
		  << " milliseconds" << std::endl;*/
  std::cout<<bytesTotal*8/(lastRxTime-firstRxTime)/1024<<" ";
  std::cout<<delaySums / rxPackets / 1000000000<<" ";
  std::cout<<bytesTotal*8/(lastRxTime-firstRxTime)/1024/nNodes<<" ";
  std::cout<<lostPackets / (rxPackets+lostPackets)<<" ";
  std::cout<<std::endl;

  return 0;
}

