import React, { useState } from 'react';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, Legend, ResponsiveContainer, AreaChart, Area } from 'recharts';
import { Battery, Gauge, Wind, Weight, Clock, Thermometer } from 'lucide-react';
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';
import { Alert, AlertDescription, AlertTitle } from '@/components/ui/alert';

// Sample data - in real implementation, this would come from your sensors/API
const generateSampleData = (points = 24) => {
  return Array.from({ length: points }, (_, i) => ({
    time: `${i}:00`,
    energy: Math.round(50 + Math.random() * 30),
    speed: Math.round(60 + Math.random() * 20),
    weight: Math.round(15000 + Math.random() * 5000),
    temperature: Math.round(20 + Math.random() * 5),
    windSpeed: Math.round(5 + Math.random() * 10),
  }));
};

const MetricCard = ({ icon: Icon, title, value, unit }) => (
  <Card className="bg-white">
    <CardContent className="pt-4">
      <div className="flex items-center justify-between">
        <div className="flex items-center space-x-2">
          <Icon className="h-6 w-6 text-blue-500" />
          <h3 className="text-sm font-medium text-gray-600">{title}</h3>
        </div>
        <div className="text-2xl font-bold">
          {value}
          <span className="text-sm font-normal text-gray-500 ml-1">{unit}</span>
        </div>
      </div>
    </CardContent>
  </Card>
);

export default function TransportDashboard() {
  const [data] = useState(generateSampleData());
  const currentData = data[data.length - 1];
  
  const efficiency = Math.round((currentData.speed / currentData.energy) * 100);
  const isEfficiencyGood = efficiency > 80;

  return (
    <div className="p-6 space-y-6 bg-gray-50 min-h-screen">
      <div className="flex justify-between items-center">
        <h1 className="text-2xl font-bold text-gray-800">運輸能源管理系統</h1>
        <div className="text-sm text-gray-500">
          最後更新: {new Date().toLocaleString()}
        </div>
      </div>

      {/* Alert for efficiency status */}
      <Alert className={isEfficiencyGood ? "bg-green-50" : "bg-yellow-50"}>
        <AlertTitle className={`text-${isEfficiencyGood ? 'green' : 'yellow'}-700`}>
          系統效率: {efficiency}%
        </AlertTitle>
        <AlertDescription>
          {isEfficiencyGood 
            ? "系統運行效率良好，保持目前的運行參數。" 
            : "建議調整運行參數以提升效率。"}
        </AlertDescription>
      </Alert>

      {/* Key Metrics Grid */}
      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 xl:grid-cols-5 gap-4">
        <MetricCard 
          icon={Gauge} 
          title="目前車速" 
          value={currentData.speed} 
          unit="km/h"
        />
        <MetricCard 
          icon={Battery} 
          title="能源消耗" 
          value={currentData.energy} 
          unit="kW"
        />
        <MetricCard 
          icon={Weight} 
          title="載重" 
          value={currentData.weight} 
          unit="kg"
        />
        <MetricCard 
          icon={Thermometer} 
          title="溫度" 
          value={currentData.temperature} 
          unit="°C"
        />
        <MetricCard 
          icon={Wind} 
          title="風速" 
          value={currentData.windSpeed} 
          unit="m/s"
        />
      </div>

      {/* Charts */}
      <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
        {/* Energy Consumption Chart */}
        <Card>
          <CardHeader>
            <CardTitle>能源消耗趨勢</CardTitle>
          </CardHeader>
          <CardContent>
            <ResponsiveContainer width="100%" height={300}>
              <LineChart data={data}>
                <CartesianGrid strokeDasharray="3 3" />
                <XAxis dataKey="time" />
                <YAxis />
                <Tooltip />
                <Legend />
                <Line 
                  type="monotone" 
                  dataKey="energy" 
                  stroke="#2563eb" 
                  name="能源消耗 (kW)"
                />
              </LineChart>
            </ResponsiveContainer>
          </CardContent>
        </Card>

        {/* Speed vs Weight Chart */}
        <Card>
          <CardHeader>
            <CardTitle>速度與載重關係</CardTitle>
          </CardHeader>
          <CardContent>
            <ResponsiveContainer width="100%" height={300}>
              <AreaChart data={data}>
                <CartesianGrid strokeDasharray="3 3" />
                <XAxis dataKey="time" />
                <YAxis yAxisId="left" />
                <YAxis yAxisId="right" orientation="right" />
                <Tooltip />
                <Legend />
                <Area 
                  type="monotone" 
                  dataKey="speed" 
                  stroke="#2563eb" 
                  fill="#93c5fd" 
                  yAxisId="left"
                  name="速度 (km/h)"
                />
                <Area 
                  type="monotone" 
                  dataKey="weight" 
                  stroke="#059669" 
                  fill="#6ee7b7" 
                  yAxisId="right"
                  name="載重 (kg)"
                />
              </AreaChart>
            </ResponsiveContainer>
          </CardContent>
        </Card>
      </div>
    </div>
  );
}