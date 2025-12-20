// Service for communicating with the backend API

class ChatbotApiService {
  constructor() {
    // const { siteConfig } = useDocusaurusContext();
    // console.log("Backend URL from config:", siteConfig.customFields.backendUrl);
    this.backendEndpoint =  'http://localhost:8000/query';

  }

  // Send a query to the backend
  async sendQuery(query) {
    console.log("Sending query to backend:", this.backendEndpoint);
    try {
      const response = await fetch(this.backendEndpoint, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ query }),
      });
      console.log("Response status:", response);
      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.error || `HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('API call failed:', error);
      throw error;
    }
  }

  // Streaming version of sendQuery (if backend supports it)
  async sendQueryStream(query, onChunk) {
    try {
      // Check if the browser supports streaming
      if (window.ReadableStream && this.backendEndpoint.includes('stream')) {
        // For true streaming implementation, we would use fetch with response.body
        // This is a simplified version that still calls the regular endpoint
        // but could be enhanced for true streaming in the future
        const result = await this.sendQuery(query);

        // If the backend supports streaming, this would be implemented differently
        // For now, we just return the full result as a single "chunk"
        if (onChunk && result.response) {
          // Simulate streaming by breaking the response into parts if needed
          onChunk(result.response);
        }

        return result;
      } else {
        // Fallback to regular API call if streaming isn't supported
        return await this.sendQuery(query);
      }
    } catch (error) {
      console.error('Streaming API call failed:', error);
      throw error;
    }
  }

  // Function to enable loading indicator while waiting for response (FR-008)
  async sendQueryWithLoading(query, setLoading) {
    try {
      setLoading(true);
      const response = await this.sendQuery(query);
      return response;
    } finally {
      setLoading(false);
    }
  }

  // Update the backend endpoint (for testing or configuration)
  setBackendEndpoint(endpoint) {
    this.backendEndpoint = endpoint;
  }

  // Get the current backend endpoint
  getBackendEndpoint() {
    return this.backendEndpoint;
  }
}

export default new ChatbotApiService();