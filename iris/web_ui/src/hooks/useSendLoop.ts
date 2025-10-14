import { useEffect } from 'react';
import axios from 'axios';
import type { DataType } from '../types';

export function useSendLoop(sending: boolean, data: Partial<DataType>) {
  useEffect(() => {
    let interval: ReturnType<typeof setInterval>;

    const sendData = async () => {
      try {
        await axios.post('http://localhost:5000/data', data, {
          headers: { 'Content-Type': 'application/json' },
        });
        console.log('Dados enviados:', data);
      } catch (error) {
        console.error('Erro ao enviar dados:', error);
      }
    };

    if (sending) {
      sendData(); // envia imediatamente
      interval = setInterval(sendData, 33); // envia a cada 33ms
    }

    return () => {
      if (interval) clearInterval(interval);
    };
  }, [sending, data]);
}
