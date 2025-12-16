"""
Thread item converter for handling different content types in ChatKit
"""
from agents import Message, Runner, ResponseInputTextParam
from chatkit.agents import AgentContext, ThreadItemConverter, stream_agent_response
from chatkit.types import Attachment, HiddenContextItem, ThreadMetadata, UserMessageItem
from chatkit.types.attachments import ImageAttachment
import base64


class MyThreadConverter(ThreadItemConverter):
    """
    Custom thread item converter to handle various content types
    """
    async def attachment_to_message_content(self, attachment: Attachment) -> ResponseInputTextParam:
        """
        Convert attachment to message content
        """
        content = await attachment_store.get_attachment_contents(attachment.id)
        mime = attachment.mime_type  # Assuming this is available
        data_url = "data:%s;base64,%s" % (mime, base64.b64encode(content).decode("utf-8"))

        if isinstance(attachment, ImageAttachment):
            from agents.sdk.params import ResponseInputImageParam
            return ResponseInputImageParam(
                type="input_image",
                detail="auto",
                image_url=data_url,
            )

        # Handle other attachment types
        # ... implementation for other types

    def hidden_context_to_input(self, item: HiddenContextItem) -> Message:
        """
        Convert hidden context to input message
        """
        return Message(
            type="message",
            role="system",
            content=[
                ResponseInputTextParam(
                    type="input_text",
                    text=f"<HIDDEN_CONTEXT>{item.content}</HIDDEN_CONTEXT>",
                )
            ],
        )

    # Additional conversion methods can be added here