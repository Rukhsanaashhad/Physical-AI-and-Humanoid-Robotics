import React, { useState, useEffect } from 'react'; // Import useEffect
import styles from './ChatWidget.module.css';
import clsx from 'clsx'; // Import clsx for conditional class names

interface Message {
  text: string;
  sender: 'user' | 'bot';
  citation?: string;
}

const CHAT_BACKEND_URL = 'http://localhost:8000';
const CONVERSATION_ID_KEY = 'rag_chatbot_conversation_id';

const ChatWidget: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const [conversationId, setConversationId] = useState<string | null>(null);

  useEffect(() => {
    // Attempt to load conversation ID from local storage
    const storedConversationId = localStorage.getItem(CONVERSATION_ID_KEY);
    if (storedConversationId) {
      setConversationId(storedConversationId);
    }

    // Function to get selected text
    const getSelectedText = () => {
      const selection = window.getSelection();
      if (selection && selection.toString().length > 0) {
        setSelectedText(selection.toString());
      } else {
        setSelectedText(null);
      }
    };

    // Add event listener for mouseup to detect text selection
    document.addEventListener('mouseup', getSelectedText);

    // Cleanup the event listener on component unmount
    return () => {
      document.removeEventListener('mouseup', getSelectedText);
    };
  }, []); // Empty dependency array means this runs once on mount and once on unmount

  // Effect to fetch history when chat opens or conversationId changes
  useEffect(() => {
    if (isOpen && conversationId && messages.length === 0) { // Only fetch if chat is open and no messages loaded
      const fetchHistory = async () => {
        try {
          const response = await fetch(`${CHAT_BACKEND_URL}/history/${conversationId}`);
          if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
          }
          const data = await response.json();
          // Map backend history format to frontend Message interface
          const historyMessages: Message[] = data.history.map((msg: any) => ({
            text: msg.message_text,
            sender: msg.sender,
            citation: msg.citation,
          }));
          setMessages(historyMessages);
        } catch (error) {
          console.error('Error fetching chat history:', error);
        }
      };
      fetchHistory();
    }
  }, [isOpen, conversationId, messages.length]); // Depend on isOpen and conversationId

  const toggleChat = () => {
    setIsOpen(!isOpen);
    // Clear selected text when chat is closed
    if (isOpen) {
      setSelectedText(null);
    }
  };

  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    setInput(e.target.value);
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (input.trim() === '') return;

    const userMessage: Message = { text: input, sender: 'user' };
    setMessages((prevMessages) => [...prevMessages, userMessage]);
    setInput('');
    setIsLoading(true);

    try {
      const body = {
        query: input,
        selected_text: selectedText,
        conversation_id: conversationId, // Include conversation ID in the request
      };

      const response = await fetch(`${CHAT_BACKEND_URL}/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(body),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      const botMessage: Message = { text: data.response, sender: 'bot', citation: data.citation };
      setMessages((prevMessages) => [...prevMessages, botMessage]);

      // If a new conversation ID is returned, save it
      if (data.conversation_id && data.conversation_id !== conversationId) {
        setConversationId(data.conversation_id);
        localStorage.setItem(CONVERSATION_ID_KEY, data.conversation_id);
      }

    } catch (error) {
      console.error('Error sending message to backend:', error);
      setMessages((prevMessages) => [...prevMessages, { text: 'Sorry, I am unable to connect to the chatbot.', sender: 'bot' }]);
    } finally {
      setIsLoading(false);
      setSelectedText(null); // Clear selected text after submission
    }
  };

  return (
    <>
      <div className={styles.chatButton} onClick={toggleChat} title="Open Chat">
        💬
      </div>

      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <span>AI Assistant</span>
            <button onClick={toggleChat}>X</button>
          </div>
          <div className={styles.chatBody}>
            {messages.length === 0 && <p className={styles.welcomeMessage}>Ask me anything about the book!</p>}
            {messages.map((msg, index) => (
              <div key={index} className={clsx(styles.message, styles[msg.sender])}>
                <p>{msg.text}</p>
                {msg.citation && <small className={styles.citation}>Source: {msg.citation}</small>} {/* Display source */}
              </div>
            ))}
            {isLoading && <div className={styles.loadingIndicator}>...typing</div>}
          </div>
          {selectedText && (
            <div className={styles.selectedTextContext}>
              Using selected context: "{selectedText.substring(0, 50)}..."
              <button onClick={() => setSelectedText(null)}>X</button>
            </div>
          )}
          <form onSubmit={handleSubmit} className={styles.chatInputForm}>
            <input
              type="text"
              value={input}
              onChange={handleInputChange}
              placeholder={selectedText ? 'Ask about selected text...' : 'Type your message...'}
              disabled={isLoading}
            />
            <button type="submit" disabled={isLoading}>Send</button>
          </form>
        </div>
      )}
    </>
  );
};

export default ChatWidget;
