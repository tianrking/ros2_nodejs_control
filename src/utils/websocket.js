// utils/websocket.js

export class WebSocketClient {
    constructor(url, options = {}) {
      this.url = url;
      this.options = options;
      this.ws = null;
      this.reconnectAttempts = 0;
      this.maxReconnectAttempts = options.maxReconnectAttempts || 5;
      this.reconnectTimeout = null;
      this.isManualClose = false;
    }
  
    connect() {
      try {
        this.ws = new WebSocket(this.url);
        this.setupEventHandlers();
      } catch (error) {
        console.error('WebSocket connection error:', error);
        this.handleReconnect();
      }
    }
  
    setupEventHandlers() {
      this.ws.onopen = () => {
        this.reconnectAttempts = 0;
        this.isManualClose = false;
        console.log('WebSocket connected');
        this.options.onOpen?.();
      };
  
      this.ws.onclose = (event) => {
        console.log('WebSocket closed:', event);
        this.options.onClose?.();
        if (!this.isManualClose) {
          this.handleReconnect();
        }
      };
  
      this.ws.onerror = (error) => {
        console.error('WebSocket error:', error);
        this.options.onError?.(error);
      };
  
      this.ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          this.options.onMessage?.(data);
        } catch (error) {
          console.error('Error parsing WebSocket message:', error);
        }
      };
    }
  
    send(data) {
      if (this.ws?.readyState === WebSocket.OPEN) {
        try {
          this.ws.send(JSON.stringify(data));
          return true;
        } catch (error) {
          console.error('Error sending WebSocket message:', error);
          return false;
        }
      }
      return false;
    }
  
    handleReconnect() {
      if (this.reconnectTimeout) {
        clearTimeout(this.reconnectTimeout);
      }
  
      if (this.reconnectAttempts < this.maxReconnectAttempts) {
        this.reconnectAttempts++;
        const delay = Math.min(1000 * Math.pow(2, this.reconnectAttempts - 1), 10000);
        
        console.log(`Attempting to reconnect (${this.reconnectAttempts}/${this.maxReconnectAttempts}) in ${delay}ms`);
        
        this.reconnectTimeout = setTimeout(() => {
          this.connect();
        }, delay);
      } else {
        console.log('Max reconnection attempts reached');
        this.options.onError?.(new Error('Max reconnection attempts reached'));
      }
    }
  
    close() {
      this.isManualClose = true;
      if (this.reconnectTimeout) {
        clearTimeout(this.reconnectTimeout);
      }
      if (this.ws) {
        this.ws.close();
      }
    }
  
    getState() {
      return this.ws ? this.ws.readyState : WebSocket.CLOSED;
    }
  }
  
  export const WebSocketState = {
    CONNECTING: WebSocket.CONNECTING,
    OPEN: WebSocket.OPEN,
    CLOSING: WebSocket.CLOSING,
    CLOSED: WebSocket.CLOSED
  };