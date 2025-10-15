import { useEffect } from 'react';
import type { DataType } from '../types';
import { sendPost } from './useSendPost';

export function useSendLoop(sending: boolean, data: Partial<DataType>) {
  useEffect(() => {
    let interval: ReturnType<typeof setInterval>;
    const url = 'http://localhost:5000/data'; // ou outro endpoint real/mock

    const sendData = async () => {
      const success = await sendPost(url, data);
      if (success) {
        console.log('✅ Dados enviados com sucesso:', data);
      } else {
        console.warn('⚠️ Falha ao enviar dados.');
      }
    };

    if (sending) {
      sendData(); // Envia imediatamente
      interval = setInterval(sendData, 33); // Envia a cada 33ms
    }

    return () => {
     if (interval) clearInterval(interval);
    };
  }, [sending, data]);
}
