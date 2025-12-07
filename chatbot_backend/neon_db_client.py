# # chatbot_backend/neon_db_client.py
# import os
# from datetime import datetime
# from typing import List, Dict, Any
# import psycopg2
# from psycopg2 import sql
# from dotenv import load_dotenv

# load_dotenv() # Load environment variables from .env file

# class NeonDBManager:
#     def __init__(self):
#         self.neon_db_url = os.getenv("NEON_DB_URL")
#         if not self.neon_db_url:
#             raise ValueError("NEON_DB_URL must be set in the .env file.")
        
#         self._conn = None # Initialize connection to None
#         self._initialize_db()

#     def _get_connection(self):
#         """Establishes and returns a database connection."""
#         if self._conn is None or self._conn.closed:
#             self._conn = psycopg2.connect(self.neon_db_url)
#         return self._conn

#     def _initialize_db(self):
#         """Creates tables if they don't exist."""
#         conn = self._get_connection()
#         with conn.cursor() as cur:
#             cur.execute("""
#                 CREATE TABLE IF NOT EXISTS conversations (
#                     conversation_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
#                     user_id VARCHAR(255),
#                     start_time TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
#                     last_active TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
#                 );
#                 CREATE TABLE IF NOT EXISTS messages (
#                     message_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
#                     conversation_id UUID NOT NULL REFERENCES conversations(conversation_id),
#                     sender VARCHAR(50) NOT NULL, -- 'user' or 'bot'
#                     message_text TEXT NOT NULL,
#                     timestamp TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
#                     citation TEXT, -- For bot responses
#                     selected_text TEXT, -- For user queries
#                     raw_query TEXT -- For user queries
#                 );
#             """)
#             conn.commit()
#         conn.close() # Close connection after initialization

#     def create_conversation(self, user_id: str = None) -> str:
#         """Creates a new conversation and returns its ID."""
#         conn = self._get_connection()
#         with conn.cursor() as cur:
#             if user_id:
#                 cur.execute("INSERT INTO conversations (user_id) VALUES (%s) RETURNING conversation_id;", (user_id,))
#             else:
#                 cur.execute("INSERT INTO conversations DEFAULT VALUES RETURNING conversation_id;")
#             conversation_id = cur.fetchone()[0]
#             conn.commit()
#         conn.close()
#         return str(conversation_id)

#     def add_message(self, conversation_id: str, sender: str, message_text: str, citation: str = None, selected_text: str = None, raw_query: str = None):
#         """Adds a message to a conversation."""
#         conn = self._get_connection()
#         with conn.cursor() as cur:
#             cur.execute(
#                 """
#                 INSERT INTO messages (conversation_id, sender, message_text, citation, selected_text, raw_query)
#                 VALUES (%s, %s, %s, %s, %s, %s);
#                 """,
#                 (conversation_id, sender, message_text, citation, selected_text, raw_query)
#             )
#             cur.execute(
#                 """
#                 UPDATE conversations SET last_active = CURRENT_TIMESTAMP WHERE conversation_id = %s;
#                 """,
#                 (conversation_id,)
#             )
#             conn.commit()
#         conn.close()

#     def get_conversation_history(self, conversation_id: str) -> List[Dict[str, Any]]:
#         """Retrieves all messages for a given conversation ID."""
#         conn = self._get_connection()
#         with conn.cursor() as cur:
#             cur.execute(
#                 """
#                 SELECT sender, message_text, timestamp, citation, selected_text, raw_query
#                 FROM messages
#                 WHERE conversation_id = %s
#                 ORDER BY timestamp;
#                 """,
#                 (conversation_id,)
#             )
#             rows = cur.fetchall()
#             conn.close()
#             # Convert rows to a list of dictionaries for easier handling
#             columns = [desc[0] for desc in cur.description]
#             history = []
#             for row in rows:
#                 history.append(dict(zip(columns, row)))
#             return history
        
#     def close(self):
#         """Closes the database connection if open."""
#         if self._conn and not self._conn.closed:
#             self._conn.close()
#             self._conn = None


# if __name__ == "__main__":
#     # Example Usage:
#     # Ensure NEON_DB_URL is set in your .env file
#     db_manager = NeonDBManager()

#     try:
#         # Create a new conversation
#         new_conv_id = db_manager.create_conversation(user_id="test_user_123")
#         print(f"Created new conversation with ID: {new_conv_id}")

#         # Add messages
#         db_manager.add_message(new_conv_id, "user", "Hello chatbot!")
#         db_manager.add_message(new_conv_id, "bot", "Hello! How can I help you?", citation="Intro (Section 1)")
#         db_manager.add_message(new_conv_id, "user", "What is RAG?", raw_query="What is RAG?")
#         db_manager.add_message(new_conv_id, "bot", "RAG combines retrieval and generation.", citation="Module 3 (Section 2)")

#         # Get history
#         history = db_manager.get_conversation_history(new_conv_id)
#         print("\nConversation History:")
#         for msg in history:
#             print(f"[{msg['timestamp'].strftime('%H:%M:%S')}] {msg['sender']}: {msg['message_text']} (Citation: {msg['citation'] if msg['citation'] else 'N/A'})")
#             if msg['selected_text']:
#                 print(f"  Selected Text: {msg['selected_text'][:50]}...")

#     except Exception as e:
#         print(f"An error occurred: {e}")
#     finally:
#         db_manager.close()
