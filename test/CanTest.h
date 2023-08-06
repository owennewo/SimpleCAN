class CanTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        can.init(500000, CanMode::CAN_LOOPBACK);
        can.start();
    }

    TEST_F(QueueTest, IsEmptyInitially)
    {
        // EXPECT_EQ(q0_.size(), 0);
    }

    void TearDown() override
    {
        can.~Can();
    }

    Can can;
};